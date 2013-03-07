#ifndef __ASM_MACH_IRQS_H
#define __ASM_MACH_IRQS_H

#include <linux/sh_intc.h>

/* GIC */
#define gic_spi(nr)		((nr) + 32)

/* INTCS */
#define INTCS_VECT_BASE		0x2200
#define INTCS_VECT(n, vect)	INTC_VECT((n), INTCS_VECT_BASE + (vect))
#define intcs_evt2irq(evt)	evt2irq(INTCS_VECT_BASE + (evt))

#define IRQ_USBH_0_OHCI (209 + 0)
#define IRQ_USBH_0_EHCI (209 + 1)
#define IRQ_USBH_1_OHCI (209 + 2)
#define IRQ_USBH_1_EHCI (209 + 3)
#define IRQ_MMC_0_ERR   (209 + 4)
#define IRQ_MMC_0_ACC   (209 + 5)
#define IRQ_MMC_1_ERR   (209 + 6)
#define IRQ_MMC_1_ACC   (209 + 7)

#define IRQ_DMAC_H(n)	(209 + 8 + (n))

#endif /* __ASM_MACH_IRQS_H */
