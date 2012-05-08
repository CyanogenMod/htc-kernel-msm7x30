/*
 *    Copyright IBM Corp. 2004,2011
 *    Author(s): Martin Schwidefsky <schwidefsky@de.ibm.com>,
 *		 Holger Smolinski <Holger.Smolinski@de.ibm.com>,
 *		 Thomas Spatzier <tspat@de.ibm.com>,
 *
 * This file contains interrupt related functions.
 */

#include <linux/kernel_stat.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/profile.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ftrace.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <asm/irq_regs.h>
#include <asm/cputime.h>
#include <asm/lowcore.h>
#include <asm/irq.h>
#include "entry.h"

struct irq_class {
	char *name;
	char *desc;
};

static const struct irq_class intrclass_names[] = {
	{.name = "EXT" },
	{.name = "I/O" },
	{.name = "CLK", .desc = "[EXT] Clock Comparator" },
	{.name = "IPI", .desc = "[EXT] Signal Processor" },
	{.name = "TMR", .desc = "[EXT] CPU Timer" },
	{.name = "TAL", .desc = "[EXT] Timing Alert" },
	{.name = "PFL", .desc = "[EXT] Pseudo Page Fault" },
	{.name = "DSD", .desc = "[EXT] DASD Diag" },
	{.name = "VRT", .desc = "[EXT] Virtio" },
	{.name = "SCP", .desc = "[EXT] Service Call" },
	{.name = "IUC", .desc = "[EXT] IUCV" },
	{.name = "CPM", .desc = "[EXT] CPU Measurement" },
	{.name = "QAI", .desc = "[I/O] QDIO Adapter Interrupt" },
	{.name = "QDI", .desc = "[I/O] QDIO Interrupt" },
	{.name = "DAS", .desc = "[I/O] DASD" },
	{.name = "C15", .desc = "[I/O] 3215" },
	{.name = "C70", .desc = "[I/O] 3270" },
	{.name = "TAP", .desc = "[I/O] Tape" },
	{.name = "VMR", .desc = "[I/O] Unit Record Devices" },
	{.name = "LCS", .desc = "[I/O] LCS" },
	{.name = "CLW", .desc = "[I/O] CLAW" },
	{.name = "CTC", .desc = "[I/O] CTC" },
	{.name = "APB", .desc = "[I/O] AP Bus" },
	{.name = "NMI", .desc = "[NMI] Machine Check" },
};

/*
 * show_interrupts is needed by /proc/interrupts.
 */
int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, j;

	get_online_cpus();
	if (i == 0) {
		seq_puts(p, "           ");
		for_each_online_cpu(j)
			seq_printf(p, "CPU%d       ",j);
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		seq_printf(p, "%s: ", intrclass_names[i].name);
#ifndef CONFIG_SMP
		seq_printf(p, "%10u ", kstat_irqs(i));
#else
		for_each_online_cpu(j)
			seq_printf(p, "%10u ", kstat_cpu(j).irqs[i]);
#endif
		if (intrclass_names[i].desc)
			seq_printf(p, "  %s", intrclass_names[i].desc);
                seq_putc(p, '\n');
        }
	put_online_cpus();
        return 0;
}

/*
 * For compatibilty only. S/390 specific setup of interrupts et al. is done
 * much later in init_channel_subsystem().
 */
void __init init_IRQ(void)
{
	/* nothing... */
}

/*
 * Switch to the asynchronous interrupt stack for softirq execution.
 */
asmlinkage void do_softirq(void)
{
	unsigned long flags, old, new;

	if (in_interrupt())
		return;

	local_irq_save(flags);

	if (local_softirq_pending()) {
		/* Get current stack pointer. */
		asm volatile("la %0,0(15)" : "=a" (old));
		/* Check against async. stack address range. */
		new = S390_lowcore.async_stack;
		if (((new - old) >> (PAGE_SHIFT + THREAD_ORDER)) != 0) {
			/* Need to switch to the async. stack. */
			new -= STACK_FRAME_OVERHEAD;
			((struct stack_frame *) new)->back_chain = old;

			asm volatile("   la    15,0(%0)\n"
				     "   basr  14,%2\n"
				     "   la    15,0(%1)\n"
				     : : "a" (new), "a" (old),
				         "a" (__do_softirq)
				     : "0", "1", "2", "3", "4", "5", "14",
				       "cc", "memory" );
		} else
			/* We are already on the async stack. */
			__do_softirq();
	}

	local_irq_restore(flags);
}

#ifdef CONFIG_PROC_FS
void init_irq_proc(void)
{
	struct proc_dir_entry *root_irq_dir;

	root_irq_dir = proc_mkdir("irq", NULL);
	create_prof_cpu_mask(root_irq_dir);
}
#endif

/*
 * ext_int_hash[index] is the start of the list for all external interrupts
 * that hash to this index. With the current set of external interrupts
 * (0x1202 external call, 0x1004 cpu timer, 0x2401 hwc console, 0x4000
 * iucv and 0x2603 pfault) this is always the first element.
 */

struct ext_int_info {
	struct ext_int_info *next;
	ext_int_handler_t handler;
	u16 code;
};

static struct ext_int_info *ext_int_hash[256];

static inline int ext_hash(u16 code)
{
	return (code + (code >> 9)) & 0xff;
}

int register_external_interrupt(u16 code, ext_int_handler_t handler)
{
	struct ext_int_info *p;
	int index;

	p = kmalloc(sizeof(*p), GFP_ATOMIC);
	if (!p)
		return -ENOMEM;
	p->code = code;
	p->handler = handler;
	index = ext_hash(code);
	p->next = ext_int_hash[index];
	ext_int_hash[index] = p;
	return 0;
}
EXPORT_SYMBOL(register_external_interrupt);

int unregister_external_interrupt(u16 code, ext_int_handler_t handler)
{
	struct ext_int_info *p, *q;
	int index;

	index = ext_hash(code);
	q = NULL;
	p = ext_int_hash[index];
	while (p) {
		if (p->code == code && p->handler == handler)
			break;
		q = p;
		p = p->next;
	}
	if (!p)
		return -ENOENT;
	if (q)
		q->next = p->next;
	else
		ext_int_hash[index] = p->next;
	kfree(p);
	return 0;
}
EXPORT_SYMBOL(unregister_external_interrupt);

void __irq_entry do_extint(struct pt_regs *regs, unsigned int ext_int_code,
			   unsigned int param32, unsigned long param64)
{
	struct pt_regs *old_regs;
	unsigned short code;
	struct ext_int_info *p;
	int index;

	code = (unsigned short) ext_int_code;
	old_regs = set_irq_regs(regs);
	s390_idle_check(regs, S390_lowcore.int_clock,
			S390_lowcore.async_enter_timer);
	irq_enter();
	if (S390_lowcore.int_clock >= S390_lowcore.clock_comparator)
		/* Serve timer interrupts first. */
		clock_comparator_work();
	kstat_cpu(smp_processor_id()).irqs[EXTERNAL_INTERRUPT]++;
	if (code != 0x1004)
		__get_cpu_var(s390_idle).nohz_delay = 1;
	index = ext_hash(code);
	for (p = ext_int_hash[index]; p; p = p->next) {
		if (likely(p->code == code))
			p->handler(ext_int_code, param32, param64);
	}
	irq_exit();
	set_irq_regs(old_regs);
}

static DEFINE_SPINLOCK(sc_irq_lock);
static int sc_irq_refcount;

void service_subclass_irq_register(void)
{
	spin_lock(&sc_irq_lock);
	if (!sc_irq_refcount)
		ctl_set_bit(0, 9);
	sc_irq_refcount++;
	spin_unlock(&sc_irq_lock);
}
EXPORT_SYMBOL(service_subclass_irq_register);

void service_subclass_irq_unregister(void)
{
	spin_lock(&sc_irq_lock);
	sc_irq_refcount--;
	if (!sc_irq_refcount)
		ctl_clear_bit(0, 9);
	spin_unlock(&sc_irq_lock);
}
EXPORT_SYMBOL(service_subclass_irq_unregister);
