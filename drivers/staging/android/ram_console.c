/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_data/ram_console.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
#include <linux/rslib.h>
#endif

#ifdef CONFIG_MDM9K_ERROR_CORRECTION
#include <mach/oem_rapi_client.h>
#include <linux/wait.h>
#include <linux/sched.h>
#endif

struct ram_console_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

#define RAM_CONSOLE_SIG (0x43474244) /* DBGC */

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
static char __initdata
	ram_console_old_log_init_buffer[CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE];
#endif
static char *ram_console_old_log;
static size_t ram_console_old_log_size;

static struct ram_console_buffer *ram_console_buffer;
static size_t ram_console_buffer_size;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
static char *ram_console_par_buffer;
static struct rs_control *ram_console_rs_decoder;
static int ram_console_corrected_bytes;
static int ram_console_bad_blocks;
#define ECC_BLOCK_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_DATA_SIZE
#define ECC_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_ECC_SIZE
#define ECC_SYMSIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_SYMBOL_SIZE
#define ECC_POLY CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_POLYNOMIAL
#endif
#ifdef CONFIG_MDM9K_ERROR_CORRECTION
#define MDM9K_BUFF_SIZE                 128
#define MDM9K_CHECK_ERROR               2002
static int RPC_READY;
struct rpc_link {
	struct delayed_work dwork;		/*check RPC ready*/
	wait_queue_head_t rpcwq;		/*wait until RPC ready*/
	struct msm_rpc_client *rpc_client;
};
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
static void ram_console_encode_rs8(uint8_t *data, size_t len, uint8_t *ecc)
{
	int i;
	uint16_t par[ECC_SIZE];
	/* Initialize the parity buffer */
	memset(par, 0, sizeof(par));
	encode_rs8(ram_console_rs_decoder, data, len, par, 0);
	for (i = 0; i < ECC_SIZE; i++)
		ecc[i] = par[i];
}

static int ram_console_decode_rs8(void *data, size_t len, uint8_t *ecc)
{
	int i;
	uint16_t par[ECC_SIZE];
	for (i = 0; i < ECC_SIZE; i++)
		par[i] = ecc[i];
	return decode_rs8(ram_console_rs_decoder, data, par, len,
				NULL, 0, NULL, 0, NULL);
}
#endif

static void ram_console_update(const char *s, unsigned int count)
{
	struct ram_console_buffer *buffer = ram_console_buffer;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	uint8_t *buffer_end = buffer->data + ram_console_buffer_size;
	uint8_t *block;
	uint8_t *par;
	int size = ECC_BLOCK_SIZE;
#endif
	memcpy(buffer->data + buffer->start, s, count);
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	block = buffer->data + (buffer->start & ~(ECC_BLOCK_SIZE - 1));
	par = ram_console_par_buffer +
	      (buffer->start / ECC_BLOCK_SIZE) * ECC_SIZE;
	do {
		if (block + ECC_BLOCK_SIZE > buffer_end)
			size = buffer_end - block;
		ram_console_encode_rs8(block, size, par);
		block += ECC_BLOCK_SIZE;
		par += ECC_SIZE;
	} while (block < buffer->data + buffer->start + count);
#endif
}

static void ram_console_update_header(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	struct ram_console_buffer *buffer = ram_console_buffer;
	uint8_t *par;
	par = ram_console_par_buffer +
	      DIV_ROUND_UP(ram_console_buffer_size, ECC_BLOCK_SIZE) * ECC_SIZE;
	ram_console_encode_rs8((uint8_t *)buffer, sizeof(*buffer), par);
#endif
}

static void
ram_console_write(struct console *console, const char *s, unsigned int count)
{
	int rem;
	struct ram_console_buffer *buffer = ram_console_buffer;

	if (count > ram_console_buffer_size) {
		s += count - ram_console_buffer_size;
		count = ram_console_buffer_size;
	}
	rem = ram_console_buffer_size - buffer->start;
	if (rem < count) {
		ram_console_update(s, rem);
		s += rem;
		count -= rem;
		buffer->start = 0;
		buffer->size = ram_console_buffer_size;
	}
	ram_console_update(s, count);

	buffer->start += count;
	if (buffer->size < ram_console_buffer_size)
		buffer->size += count;
	ram_console_update_header();
}

static struct console ram_console = {
	.name	= "ram",
	.write	= ram_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED,
	.index	= -1,
};

void ram_console_enable_console(int enabled)
{
	if (enabled)
		ram_console.flags |= CON_ENABLED;
	else
		ram_console.flags &= ~CON_ENABLED;
}

static void __init
ram_console_save_old(struct ram_console_buffer *buffer, const char *bootinfo,
	char *dest)
{
	size_t old_log_size = buffer->size;
	size_t bootinfo_size = 0;
	size_t total_size = old_log_size;
	char *ptr;
	const char *bootinfo_label = "Boot info:\n";

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	uint8_t *block;
	uint8_t *par;
	char strbuf[80];
	int strbuf_len = 0;

	block = buffer->data;
	par = ram_console_par_buffer;
	while (block < buffer->data + buffer->size) {
		int numerr;
		int size = ECC_BLOCK_SIZE;
		if (block + size > buffer->data + ram_console_buffer_size)
			size = buffer->data + ram_console_buffer_size - block;
		numerr = ram_console_decode_rs8(block, size, par);
		if (numerr > 0) {
#if 0
			printk(KERN_INFO "ram_console: error in block %p, %d\n",
			       block, numerr);
#endif
			ram_console_corrected_bytes += numerr;
		} else if (numerr < 0) {
#if 0
			printk(KERN_INFO "ram_console: uncorrectable error in "
			       "block %p\n", block);
#endif
			ram_console_bad_blocks++;
		}
		block += ECC_BLOCK_SIZE;
		par += ECC_SIZE;
	}
	if (ram_console_corrected_bytes || ram_console_bad_blocks)
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
			"\n%d Corrected bytes, %d unrecoverable blocks\n",
			ram_console_corrected_bytes, ram_console_bad_blocks);
	else
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
				      "\nNo errors detected\n");
	if (strbuf_len >= sizeof(strbuf))
		strbuf_len = sizeof(strbuf) - 1;
	total_size += strbuf_len;
#endif

	if (bootinfo)
		bootinfo_size = strlen(bootinfo) + strlen(bootinfo_label);
	total_size += bootinfo_size;

	if (dest == NULL) {
		dest = kmalloc(total_size, GFP_KERNEL);
		if (dest == NULL) {
			printk(KERN_ERR
			       "ram_console: failed to allocate buffer\n");
			return;
		}
	}

	ram_console_old_log = dest;
	ram_console_old_log_size = total_size;
	memcpy(ram_console_old_log,
	       &buffer->data[buffer->start], buffer->size - buffer->start);
	memcpy(ram_console_old_log + buffer->size - buffer->start,
	       &buffer->data[0], buffer->start);
	ptr = ram_console_old_log + old_log_size;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	memcpy(ptr, strbuf, strbuf_len);
	ptr += strbuf_len;
#endif
	if (bootinfo) {
		memcpy(ptr, bootinfo_label, strlen(bootinfo_label));
		ptr += strlen(bootinfo_label);
		memcpy(ptr, bootinfo, bootinfo_size);
		ptr += bootinfo_size;
	}
}

static int __init ram_console_init(struct ram_console_buffer *buffer,
				   size_t buffer_size, const char *bootinfo,
				   char *old_buf)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	int numerr;
	uint8_t *par;
#endif
	ram_console_buffer = buffer;
	ram_console_buffer_size =
		buffer_size - sizeof(struct ram_console_buffer);

	if (ram_console_buffer_size > buffer_size) {
		pr_err("ram_console: buffer %p, invalid size %zu, "
		       "datasize %zu\n", buffer, buffer_size,
		       ram_console_buffer_size);
		return 0;
	}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	ram_console_buffer_size -= (DIV_ROUND_UP(ram_console_buffer_size,
						ECC_BLOCK_SIZE) + 1) * ECC_SIZE;

	if (ram_console_buffer_size > buffer_size) {
		pr_err("ram_console: buffer %p, invalid size %zu, "
		       "non-ecc datasize %zu\n",
		       buffer, buffer_size, ram_console_buffer_size);
		return 0;
	}

	ram_console_par_buffer = buffer->data + ram_console_buffer_size;


	/* first consecutive root is 0
	 * primitive element to generate roots = 1
	 */
	ram_console_rs_decoder = init_rs(ECC_SYMSIZE, ECC_POLY, 0, 1, ECC_SIZE);
	if (ram_console_rs_decoder == NULL) {
		printk(KERN_INFO "ram_console: init_rs failed\n");
		return 0;
	}

	ram_console_corrected_bytes = 0;
	ram_console_bad_blocks = 0;

	par = ram_console_par_buffer +
	      DIV_ROUND_UP(ram_console_buffer_size, ECC_BLOCK_SIZE) * ECC_SIZE;

	numerr = ram_console_decode_rs8(buffer, sizeof(*buffer), par);
	if (numerr > 0) {
		printk(KERN_INFO "ram_console: error in header, %d\n", numerr);
		ram_console_corrected_bytes += numerr;
	} else if (numerr < 0) {
		printk(KERN_INFO
		       "ram_console: uncorrectable error in header\n");
		ram_console_bad_blocks++;
	}
#endif

	if (buffer->sig == RAM_CONSOLE_SIG) {
		if (buffer->size > ram_console_buffer_size
		    || buffer->start > buffer->size)
			printk(KERN_INFO "ram_console: found existing invalid "
			       "buffer, size %d, start %d\n",
			       buffer->size, buffer->start);
		else {
			printk(KERN_INFO "ram_console: found existing buffer, "
			       "size %d, start %d\n",
			       buffer->size, buffer->start);
			ram_console_save_old(buffer, bootinfo, old_buf);
		}
	} else {
		printk(KERN_INFO "ram_console: no valid data in buffer "
		       "(sig = 0x%08x)\n", buffer->sig);
	}

	buffer->sig = RAM_CONSOLE_SIG;
	buffer->start = 0;
	buffer->size = 0;

	register_console(&ram_console);
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ENABLE_VERBOSE
	console_verbose();
#endif
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
static int __init ram_console_early_init(void)
{
	return ram_console_init((struct ram_console_buffer *)
		CONFIG_ANDROID_RAM_CONSOLE_EARLY_ADDR,
		CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE,
		NULL,
		ram_console_old_log_init_buffer);
}
#else
static int ram_console_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	const char *bootinfo = NULL;
	struct ram_console_platform_data *pdata = pdev->dev.platform_data;

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "ram_console: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "ram_console: got buffer at %zx, size %zx\n",
	       start, buffer_size);
	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "ram_console: failed to map memory\n");
		return -ENOMEM;
	}

	if (pdata)
		bootinfo = pdata->bootinfo;

	return ram_console_init(buffer, buffer_size, bootinfo, NULL/* allocate */);
}

static struct platform_driver ram_console_driver = {
	.probe = ram_console_driver_probe,
	.driver		= {
		.name	= "ram_console",
	},
};

static int __init ram_console_module_init(void)
{
	int err;
	err = platform_driver_register(&ram_console_driver);
	return err;
}
#endif

#ifdef CONFIG_MDM9K_ERROR_CORRECTION
/*
 * check rpc link
 * stop checking if 1. link establishs. 2. link did not establish after 35 seconds.
 */
static void rpc_check_func(struct work_struct *work)
{
	struct rpc_link *rpc;
	static int count = 0;

	if (count++ >= 35) {
		printk(KERN_ERR "MDM9K_ERROR_CORRECTION fail due to RPC connection is not ready\n");
		return;
	}

	rpc = container_of(work, struct rpc_link, dwork.work);
	rpc->rpc_client = oem_rapi_client_init();

	if (IS_ERR(rpc->rpc_client)) {
		schedule_delayed_work(&rpc->dwork, msecs_to_jiffies(1000));
		return;
	} else {
		RPC_READY = 1;
		wake_up(&rpc->rpcwq);
	}
}

/*
 * Get error message form mdm9k.
 * MDM9K_CHECK_ERROR, and input number(0,1) are confirmed by radio team.
 */
void query_error_message(struct msm_rpc_client *rpc_client, char *buf, int check_number)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	int err, ret_len;
	char input;

	err = 0;
	ret_len = MDM9K_BUFF_SIZE;
	input = check_number;
	arg.event = MDM9K_CHECK_ERROR;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = 1;
	arg.input = &input;
	arg.out_len_valid = 1;
	arg.output_valid = 1;
	arg.output_size = MDM9K_BUFF_SIZE;
	ret.out_len = &ret_len;
	ret.output = NULL;
	err = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
	if (err) {
		printk(KERN_ERR "ram_console: Receive data from modem failed: err = %d\n", err);
	} else if (!*ret.out_len) {
		if (check_number == 0)
			strncat(buf, "[SQA][ARM] no error occur\n", MDM9K_BUFF_SIZE);
		else if (check_number == 1)
			strncat(buf, "[SQA][QDSP6] no error occur\n", MDM9K_BUFF_SIZE);
		printk(KERN_INFO "ram_console: query mdm9k message %d - out_len = 0\n", check_number);
		kfree(ret.out_len);
	} else {
		printk(KERN_INFO "ram_console: query mdm9k message %d - out_len = %d\n", check_number, *ret.out_len);
		if (check_number == 0)
			strncpy(buf, ret.output, *ret.out_len);
		else if (check_number == 1)
			strncat(buf, ret.output, *ret.out_len);
		kfree(ret.out_len);
		kfree(ret.output);
	}
}
/*
 * Put error message in buf, and return buf length.
 * due to RPC link need a long time to create (35~50 seconds).
 * rpc_check_func() is used to check the link every 1 second(stop after 35 retries).
 */
int get_mdm9k_error_message(char *buf)
{
	struct rpc_link rpc;

	rpc.rpc_client = ERR_PTR(-ENOMEM);

	if (RPC_READY) {
		rpc.rpc_client = oem_rapi_client_init();
	} else {
		INIT_DELAYED_WORK(&rpc.dwork, rpc_check_func);
		schedule_delayed_work(&rpc.dwork, msecs_to_jiffies(25000));
		init_waitqueue_head(&rpc.rpcwq);
		wait_event_timeout(rpc.rpcwq, RPC_READY == 1, msecs_to_jiffies(70000));
		flush_delayed_work(&rpc.dwork); /* avoid RPC_READY is set by others after schedule rpc.dwork */
	}

	if (IS_ERR(rpc.rpc_client)) {
		strcpy(buf, "[mdm9k] MDM9K_ERROR_CORRECTION fail due to RPC link is not ready\n");
		return strlen(buf);
	}

	printk(KERN_INFO "ram_console: RPC client ready...\n");
	query_error_message(rpc.rpc_client, buf, 0);
	query_error_message(rpc.rpc_client, buf, 1);
	oem_rapi_client_close();
	return strlen(buf);
}
#endif

static ssize_t ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

#ifdef CONFIG_MDM9K_ERROR_CORRECTION
	if (pos == ram_console_old_log_size) {
		char mdm9k_buf[256];
		printk(KERN_INFO "ram_console: MDM9K error log collection start...\n");
		memset(mdm9k_buf, 0, 256);
		count = get_mdm9k_error_message(mdm9k_buf);
		if (count == 0)
			return 0;
		if (copy_to_user(buf, mdm9k_buf, count))
			return -EFAULT;

		*offset += count;
		printk(KERN_INFO "ram_console: MDM9K error log collection stop...\n");
		return count;
	}
#endif

	if (pos >= ram_console_old_log_size)
		return 0;

	count = min(len, (size_t)(ram_console_old_log_size - pos));
	if (copy_to_user(buf, ram_console_old_log + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = ram_console_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;

	if (ram_console_old_log == NULL)
		return 0;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
	ram_console_old_log = kmalloc(ram_console_old_log_size, GFP_KERNEL);
	if (ram_console_old_log == NULL) {
		printk(KERN_ERR
		       "ram_console: failed to allocate buffer for old log\n");
		ram_console_old_log_size = 0;
		return 0;
	}
	memcpy(ram_console_old_log,
	       ram_console_old_log_init_buffer, ram_console_old_log_size);
#endif
	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "ram_console: failed to create proc entry\n");
		kfree(ram_console_old_log);
		ram_console_old_log = NULL;
		return 0;
	}

	entry->proc_fops = &ram_console_file_ops;
	entry->size = ram_console_old_log_size;
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
console_initcall(ram_console_early_init);
#else
postcore_initcall(ram_console_module_init);
#endif
late_initcall(ram_console_late_init);

