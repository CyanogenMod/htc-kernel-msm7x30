/*
 * linux/arch/arm/mach-omap2/irq.c
 *
 * Interrupt handler for OMAP2 boards.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/mach/irq.h>


/* selected INTC register offsets */

#define INTC_REVISION		0x0000
#define INTC_SYSCONFIG		0x0010
#define INTC_SYSSTATUS		0x0014
#define INTC_SIR		0x0040
#define INTC_CONTROL		0x0048
#define INTC_PROTECTION		0x004C
#define INTC_IDLE		0x0050
#define INTC_THRESHOLD		0x0068
#define INTC_MIR0		0x0084
#define INTC_MIR_CLEAR0		0x0088
#define INTC_MIR_SET0		0x008c
#define INTC_PENDING_IRQ0	0x0098
/* Number of IRQ state bits in each MIR register */
#define IRQ_BITS_PER_REG	32

/*
 * OMAP2 has a number of different interrupt controllers, each interrupt
 * controller is identified as its own "bank". Register definitions are
 * fairly consistent for each bank, but not all registers are implemented
 * for each bank.. when in doubt, consult the TRM.
 */
static struct omap_irq_bank {
	void __iomem *base_reg;
	unsigned int nr_irqs;
} __attribute__ ((aligned(4))) irq_banks[] = {
	{
		/* MPU INTC */
		.nr_irqs	= 96,
	},
};

/* Structure to save interrupt controller context */
struct omap3_intc_regs {
	u32 sysconfig;
	u32 protection;
	u32 idle;
	u32 threshold;
	u32 ilr[INTCPS_NR_IRQS];
	u32 mir[INTCPS_NR_MIR_REGS];
};

/* INTC bank register get/set */

static void intc_bank_write_reg(u32 val, struct omap_irq_bank *bank, u16 reg)
{
	__raw_writel(val, bank->base_reg + reg);
}

static u32 intc_bank_read_reg(struct omap_irq_bank *bank, u16 reg)
{
	return __raw_readl(bank->base_reg + reg);
}

/* XXX: FIQ and additional INTC support (only MPU at the moment) */
static void omap_ack_irq(struct irq_data *d)
{
	intc_bank_write_reg(0x1, &irq_banks[0], INTC_CONTROL);
}

static void omap_mask_ack_irq(struct irq_data *d)
{
	irq_gc_mask_disable_reg(d);
	omap_ack_irq(d);
}

static void __init omap_irq_bank_init_one(struct omap_irq_bank *bank)
{
	unsigned long tmp;

	tmp = intc_bank_read_reg(bank, INTC_REVISION) & 0xff;
	printk(KERN_INFO "IRQ: Found an INTC at 0x%p "
			 "(revision %ld.%ld) with %d interrupts\n",
			 bank->base_reg, tmp >> 4, tmp & 0xf, bank->nr_irqs);

	tmp = intc_bank_read_reg(bank, INTC_SYSCONFIG);
	tmp |= 1 << 1;	/* soft reset */
	intc_bank_write_reg(tmp, bank, INTC_SYSCONFIG);

	while (!(intc_bank_read_reg(bank, INTC_SYSSTATUS) & 0x1))
		/* Wait for reset to complete */;

	/* Enable autoidle */
	intc_bank_write_reg(1 << 0, bank, INTC_SYSCONFIG);
}

int omap_irq_pending(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(irq_banks); i++) {
		struct omap_irq_bank *bank = irq_banks + i;
		int irq;

		for (irq = 0; irq < bank->nr_irqs; irq += 32)
			if (intc_bank_read_reg(bank, INTC_PENDING_IRQ0 +
					       ((irq >> 5) << 5)))
				return 1;
	}
	return 0;
}

static __init void
omap_alloc_gc(void __iomem *base, unsigned int irq_start, unsigned int num)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;

	gc = irq_alloc_generic_chip("INTC", 1, irq_start, base,
					handle_level_irq);
	ct = gc->chip_types;
	ct->chip.irq_ack = omap_mask_ack_irq;
	ct->chip.irq_mask = irq_gc_mask_disable_reg;
	ct->chip.irq_unmask = irq_gc_unmask_enable_reg;

	ct->regs.ack = INTC_CONTROL;
	ct->regs.enable = INTC_MIR_CLEAR0;
	ct->regs.disable = INTC_MIR_SET0;
	irq_setup_generic_chip(gc, IRQ_MSK(num), IRQ_GC_INIT_MASK_CACHE,
				IRQ_NOREQUEST | IRQ_NOPROBE, 0);
}

void __init omap_init_irq(void)
{
	unsigned long nr_of_irqs = 0;
	unsigned int nr_banks = 0;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(irq_banks); i++) {
		unsigned long base = 0;
		struct omap_irq_bank *bank = irq_banks + i;

		if (cpu_is_omap24xx())
			base = OMAP24XX_IC_BASE;
		else if (cpu_is_omap34xx())
			base = OMAP34XX_IC_BASE;

		BUG_ON(!base);

		if (cpu_is_ti816x())
			bank->nr_irqs = 128;

		/* Static mapping, never released */
		bank->base_reg = ioremap(base, SZ_4K);
		if (!bank->base_reg) {
			printk(KERN_ERR "Could not ioremap irq bank%i\n", i);
			continue;
		}

		omap_irq_bank_init_one(bank);

		for (i = 0, j = 0; i < bank->nr_irqs; i += 32, j += 0x20)
			omap_alloc_gc(bank->base_reg + j, i, 32);

		nr_of_irqs += bank->nr_irqs;
		nr_banks++;
	}

	printk(KERN_INFO "Total of %ld interrupts on %d active controller%s\n",
	       nr_of_irqs, nr_banks, nr_banks > 1 ? "s" : "");
}

#ifdef CONFIG_ARCH_OMAP3
static struct omap3_intc_regs intc_context[ARRAY_SIZE(irq_banks)];

void omap_intc_save_context(void)
{
	int ind = 0, i = 0;
	for (ind = 0; ind < ARRAY_SIZE(irq_banks); ind++) {
		struct omap_irq_bank *bank = irq_banks + ind;
		intc_context[ind].sysconfig =
			intc_bank_read_reg(bank, INTC_SYSCONFIG);
		intc_context[ind].protection =
			intc_bank_read_reg(bank, INTC_PROTECTION);
		intc_context[ind].idle =
			intc_bank_read_reg(bank, INTC_IDLE);
		intc_context[ind].threshold =
			intc_bank_read_reg(bank, INTC_THRESHOLD);
		for (i = 0; i < INTCPS_NR_IRQS; i++)
			intc_context[ind].ilr[i] =
				intc_bank_read_reg(bank, (0x100 + 0x4*i));
		for (i = 0; i < INTCPS_NR_MIR_REGS; i++)
			intc_context[ind].mir[i] =
				intc_bank_read_reg(&irq_banks[0], INTC_MIR0 +
				(0x20 * i));
	}
}

void omap_intc_restore_context(void)
{
	int ind = 0, i = 0;

	for (ind = 0; ind < ARRAY_SIZE(irq_banks); ind++) {
		struct omap_irq_bank *bank = irq_banks + ind;
		intc_bank_write_reg(intc_context[ind].sysconfig,
					bank, INTC_SYSCONFIG);
		intc_bank_write_reg(intc_context[ind].sysconfig,
					bank, INTC_SYSCONFIG);
		intc_bank_write_reg(intc_context[ind].protection,
					bank, INTC_PROTECTION);
		intc_bank_write_reg(intc_context[ind].idle,
					bank, INTC_IDLE);
		intc_bank_write_reg(intc_context[ind].threshold,
					bank, INTC_THRESHOLD);
		for (i = 0; i < INTCPS_NR_IRQS; i++)
			intc_bank_write_reg(intc_context[ind].ilr[i],
				bank, (0x100 + 0x4*i));
		for (i = 0; i < INTCPS_NR_MIR_REGS; i++)
			intc_bank_write_reg(intc_context[ind].mir[i],
				 &irq_banks[0], INTC_MIR0 + (0x20 * i));
	}
	/* MIRs are saved and restore with other PRCM registers */
}

void omap3_intc_suspend(void)
{
	/* A pending interrupt would prevent OMAP from entering suspend */
	omap_ack_irq(0);
}

void omap3_intc_prepare_idle(void)
{
	/*
	 * Disable autoidle as it can stall interrupt controller,
	 * cf. errata ID i540 for 3430 (all revisions up to 3.1.x)
	 */
	intc_bank_write_reg(0, &irq_banks[0], INTC_SYSCONFIG);
}

void omap3_intc_resume_idle(void)
{
	/* Re-enable autoidle */
	intc_bank_write_reg(1, &irq_banks[0], INTC_SYSCONFIG);
}
#endif /* CONFIG_ARCH_OMAP3 */
