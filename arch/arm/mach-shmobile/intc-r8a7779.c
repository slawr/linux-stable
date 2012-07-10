/*
 * r8a7779 processor support - INTC hardware block
 *
 * Copyright (C) 2011  Renesas Solutions Corp.
 * Copyright (C) 2011  Magnus Damm
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/common.h>
#include <mach/intc.h>
#include <mach/r8a7779.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>

#define INT2SMSKCR0 0xfe7822a0
#define INT2SMSKCR1 0xfe7822a4
#define INT2SMSKCR2 0xfe7822a8
#define INT2SMSKCR3 0xfe7822ac
#define INT2SMSKCR4 0xfe7822b0

#define INT2NTSR0 0xfe700060
#define INT2NTSR1 0xfe700064

struct intc2_child {
	unsigned long	mask;
	unsigned int	irq;
};

struct intc2_parent {
	unsigned int		irq;
	void __iomem		*reg;
	unsigned int		num_child;
	struct intc2_child	*child;
};

/*
 * Parents
 */
static struct intc2_parent intc2_parent_desc[] = {
};

static void intc2_demux(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_get_chip(irq);
	struct intc2_parent *parent =
		(struct intc2_parent *)irq_get_handler_data(irq);
	unsigned int intr_stat = readl(parent->reg);
	struct intc2_child *child;
	int i;

	chained_irq_enter(chip, desc);

	child = parent->child;
	for (i = 0; i < parent->num_child; i++) {
		if (intr_stat & child->mask)
			generic_handle_irq(child->irq);
		child++;
	}

	chained_irq_exit(chip, desc);
}

static void intc2_irq_ack(struct irq_data *data) {}
static void intc2_irq_mask(struct irq_data *data) {}
static void intc2_irq_unmask(struct irq_data *data) {}

static struct irq_chip intc2_chip = {
	.name		= "INTC2",
	.irq_ack	= intc2_irq_ack,
	.irq_mask	= intc2_irq_mask,
	.irq_unmask	= intc2_irq_unmask,
};

static int r8a7779_set_wake(struct irq_data *data, unsigned int on)
{
	return 0; /* always allow wakeup */
}

void __init r8a7779_init_irq(void)
{
	void __iomem *gic_dist_base = IOMEM(0xf0001000);
	void __iomem *gic_cpu_base = IOMEM(0xf0000100);
	struct intc2_parent *parent;
	struct intc2_child *child;
	unsigned int irq;
	int i, j;

	gic_dist_base = IOMEM(0xf0001000);
	gic_cpu_base = IOMEM(0xf0000100);

	/* use GIC to handle interrupts */
	gic_init(0, 29, gic_dist_base, gic_cpu_base);

	gic_arch_extn.irq_set_wake = r8a7779_set_wake;

	parent = intc2_parent_desc;
	for (i = 0; i < ARRAY_SIZE(intc2_parent_desc); i++) {
		child = parent->child;
		for (j = 0; j < parent->num_child; j++) {
			irq = child->irq;
			irq_set_chip_and_handler(irq, &intc2_chip,
				handle_simple_irq);
			set_irq_flags(irq, IRQF_VALID);
			child++;
		}
		irq = parent->irq;
		irq_set_handler_data(irq, (void *)parent);
		irq_set_chained_handler(irq, intc2_demux);
		parent++;
	}

	/* route all interrupts to ARM */
	__raw_writel(0xffffffff, INT2NTSR0);
	__raw_writel(0x3fffffff, INT2NTSR1);

	/* unmask all known interrupts in INTCS2 */
	__raw_writel(0xfffffff0, INT2SMSKCR0);
	__raw_writel(0xfff7ffff, INT2SMSKCR1);
	__raw_writel(0xfffbffdf, INT2SMSKCR2);
	__raw_writel(0xbffffffc, INT2SMSKCR3);
	__raw_writel(0x003fee3f, INT2SMSKCR4);
}
