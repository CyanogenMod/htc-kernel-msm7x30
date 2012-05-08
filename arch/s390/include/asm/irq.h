#ifndef _ASM_IRQ_H
#define _ASM_IRQ_H

#include <linux/hardirq.h>
#include <linux/types.h>

enum interruption_class {
	EXTERNAL_INTERRUPT,
	IO_INTERRUPT,
	EXTINT_CLK,
	EXTINT_IPI,
	EXTINT_TMR,
	EXTINT_TLA,
	EXTINT_PFL,
	EXTINT_DSD,
	EXTINT_VRT,
	EXTINT_SCP,
	EXTINT_IUC,
	EXTINT_CPM,
	IOINT_QAI,
	IOINT_QDI,
	IOINT_DAS,
	IOINT_C15,
	IOINT_C70,
	IOINT_TAP,
	IOINT_VMR,
	IOINT_LCS,
	IOINT_CLW,
	IOINT_CTC,
	IOINT_APB,
	NMI_NMI,
	NR_IRQS,
};

typedef void (*ext_int_handler_t)(unsigned int, unsigned int, unsigned long);

int register_external_interrupt(u16 code, ext_int_handler_t handler);
int unregister_external_interrupt(u16 code, ext_int_handler_t handler);
void service_subclass_irq_register(void);
void service_subclass_irq_unregister(void);

#endif /* _ASM_IRQ_H */
