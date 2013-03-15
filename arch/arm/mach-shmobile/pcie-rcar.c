/*
 * PCIe functions for r8a7779 (R-Car H1) SoC
 *  Copyright (C) 2013 Renesas Electronics Europe Ltd
 *
 * Based on:
 *  arch/sh/drivers/pci/pcie-sh7786.c
 *  arch/sh/drivers/pci/ops-sh7786.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/pci.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include <asm/mach/pci.h>
#include <pcie-rcar.h>

static struct resource rcar_pci0_resources[] = {
	{
		.name	= "PCIe0 IO",
		.start	= 0xfe100000,
		.end	= 0xfe100000 + SZ_1M - 1,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= "PCIe0 MEM 1",
		.start	= 0xfe200000,
		.end	= 0xfe200000 + SZ_2M - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "PCIe0 MEM 2",
		.start	= 0x20000000,
		.end	= 0x20000000 + SZ_512M - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "PCIe0 MEM 3",
		.start	= 0xc0000000,
		.end	= 0xc0000000 + SZ_512M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct rcar_pcie_port {
	int			index;
	u8			root_bus_nr;
	void __iomem		*base;
	int			endpoint;
	struct resource		*resources;
	int			nr_resources;
};

struct rcar_pcie_info {
	struct rcar_pcie_port	port[1];
	int			num_ports;
	void __iomem		*regs;
	int			irq;
	struct clk		*pciec_clk;
};

static struct rcar_pcie_info rcar_pcie = {};

DEFINE_RAW_SPINLOCK(pci_config_lock);


static struct rcar_pcie_port *bus_to_port(int bus)
{
	int i;

	for (i = rcar_pcie.num_ports - 1; i >= 0; i--) {
		int rbus = rcar_pcie.port[i].root_bus_nr;
		if (rbus != -1 && rbus == bus)
			break;
	}

	return i >= 0 ? rcar_pcie.port + i : NULL;
}

static void
pci_write_reg(struct rcar_pcie_port *chan, unsigned long val, unsigned long reg)
{
	iowrite32(val, chan->base + reg);
}

static unsigned long
pci_read_reg(struct rcar_pcie_port *chan, unsigned long reg)
{
	return ioread32(chan->base + reg);
}



enum {
	PCI_ACCESS_READ,
	PCI_ACCESS_WRITE,
};

static int rcar_pcie_config_access(unsigned char access_type,
		struct pci_bus *bus, unsigned int devfn, int where, u32 *data)
{
	struct rcar_pcie_port *chan = bus_to_port(bus->number);
	int dev, func, type, reg;

	dev = PCI_SLOT(devfn);
	func = PCI_FUNC(devfn);
	type = !!bus->parent;
	reg = where & ~3;

	if (bus->number > 255 || dev > 31 || func > 7)
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	/*
	 * While each channel has its own memory-mapped extended config
	 * space, it's generally only accessible when in endpoint mode.
	 * When in root complex mode, the controller is unable to target
	 * itself with either type 0 or type 1 accesses, and indeed, any
	 * controller initiated target transfer to its own config space
	 * result in a completer abort.
	 *
	 * Each channel effectively only supports a single device, but as
	 * the same channel <-> device access works for any PCI_SLOT()
	 * value, we cheat a bit here and bind the controller's config
	 * space to devfn 0 in order to enable self-enumeration. In this
	 * case the regular PAR/PDR path is sidelined and the mangled
	 * config access itself is initiated as a SuperHyway transaction.
	 */
	if (pci_is_root_bus(bus)) {
		if (dev == 0) {
			if (reg > 0xA8/4 && reg < 0x100/4)
				return PCIBIOS_DEVICE_NOT_FOUND;

			if (access_type == PCI_ACCESS_READ)
				*data = pci_read_reg(chan, RCAR_PCIEPCICONF(reg));
			else
				pci_write_reg(chan, *data, RCAR_PCIEPCICONF(reg));

			return PCIBIOS_SUCCESSFUL;
		} else if (dev > 1)
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* Clear errors */
	pci_write_reg(chan, pci_read_reg(chan, RCAR_PCIEERRFR), RCAR_PCIEERRFR);

	/* Set the PIO address */
	pci_write_reg(chan, (bus->number << 24) | (dev << 19) |
				(func << 16) | reg, RCAR_PCIEPAR);

	/* Enable the configuration access */
	pci_write_reg(chan, MASK_CCIE | (type << BITS_TYPE), RCAR_PCIEPCTLR);

	/* Check for errors */
	if (pci_read_reg(chan, RCAR_PCIEERRFR) & 0x10)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/* Check for master and target aborts */
	if (pci_read_reg(chan, RCAR_PCIEPCICONF(1)) & ((1 << 29) | (1 << 28)))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (access_type == PCI_ACCESS_READ)
		*data = pci_read_reg(chan, RCAR_PCIEPDR);
	else
		pci_write_reg(chan, *data, RCAR_PCIEPDR);

	/* Disable the configuration access */
	pci_write_reg(chan, 0, RCAR_PCIEPCTLR);

	return PCIBIOS_SUCCESSFUL;
}

static int rcar_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	unsigned long flags;
	int ret;
	u32 data;

        if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	raw_spin_lock_irqsave(&pci_config_lock, flags);
	ret = rcar_pcie_config_access(PCI_ACCESS_READ, bus,
					devfn, where, &data);
	if (ret != PCIBIOS_SUCCESSFUL) {
		*val = 0xffffffff;
		goto out;
	}

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 2) << 3)) & 0xffff;
	else
		*val = data;

	dev_dbg(&bus->dev, "pcie-config-read: bus=%3d devfn=0x%04x "
		"where=0x%04x size=%d val=0x%08lx\n", bus->number,
		devfn, where, size, (unsigned long)*val);

out:
	raw_spin_unlock_irqrestore(&pci_config_lock, flags);
	return ret;
}

static int rcar_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	unsigned long flags;
	int shift, ret;
	u32 data;

        if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	raw_spin_lock_irqsave(&pci_config_lock, flags);
	ret = rcar_pcie_config_access(PCI_ACCESS_READ, bus,
					devfn, where, &data);
	if (ret != PCIBIOS_SUCCESSFUL)
		goto out;

	dev_dbg(&bus->dev, "pcie-config-write: bus=%3d devfn=0x%04x "
		"where=0x%04x size=%d val=%08lx\n", bus->number,
		devfn, where, size, (unsigned long)val);

	if (size == 1) {
		shift = (where & 3) << 3;
		data &= ~(0xff << shift);
		data |= ((val & 0xff) << shift);
	} else if (size == 2) {
		shift = (where & 2) << 3;
		data &= ~(0xffff << shift);
		data |= ((val & 0xffff) << shift);
	} else
		data = val;

	ret = rcar_pcie_config_access(PCI_ACCESS_WRITE, bus,
					devfn, where, &data);
out:
	raw_spin_unlock_irqrestore(&pci_config_lock, flags);
	return ret;
}

static struct pci_ops rcar_pcie_ops = {
	.read	= rcar_pcie_read_conf,
	.write	= rcar_pcie_write_conf,
};


static void __devinit rc_pci_fixup(struct pci_dev *dev)
{
	/*
	 * Prevent enumeration of root complex.
	 */
	if (dev->bus->parent == NULL && dev->devfn == 0) {
		int i;

		for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
			dev->resource[i].start = 0;
			dev->resource[i].end   = 0;
			dev->resource[i].flags = 0;
		}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_RENESAS, PCI_DEVICE_ID_RENESAS_RCAR, rc_pci_fixup);

static int rcar_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct rcar_pcie_port *pp;
	int i, win;

	if (nr >= rcar_pcie.num_ports)
		return 0;

	pp = rcar_pcie.port + nr;
	pp->root_bus_nr = sys->busnr;

	pp->resources = rcar_pci0_resources;
	pp->nr_resources = ARRAY_SIZE(rcar_pci0_resources);

	for (i=0; i<pp->nr_resources; i++) {
		struct resource *res = &rcar_pci0_resources[i];

		if (res->flags & IORESOURCE_IO) {
			if (request_resource(&ioport_resource, res) < 0)
				goto out;
		} else {
			if (request_resource(&iomem_resource, res) < 0)
				goto out;
		}
	}

	for (i=0; i<pp->nr_resources; i++) {
		struct resource *res = &rcar_pci0_resources[i];

		if (res->flags == IORESOURCE_IO)
			pci_add_resource_offset(&sys->resources,
				res, sys->io_offset);
		else
			pci_add_resource_offset(&sys->resources,
				res, sys->mem_offset);
	}

	for (i = win = 0; i < pp->nr_resources; i++) {
		struct resource *res = pp->resources + i;
		resource_size_t size;
		u32 mask;

		pci_write_reg(pp, 0x00000000, RCAR_PCIEPTCTLR(win));

		/*
		 * The PAMR mask is calculated in units of 128Bytes, which
		 * keeps things pretty simple.
		 */
		size = resource_size(res);
		mask = (roundup_pow_of_two(size) / SZ_128) - 1;
		pci_write_reg(pp, mask << 7, RCAR_PCIEPAMR(win));

		pci_write_reg(pp, upper_32_bits(res->start),
			      RCAR_PCIEPARH(win));
		pci_write_reg(pp, lower_32_bits(res->start),
			      RCAR_PCIEPARL(win));

		mask = MASK_PARE;
		if (res->flags & IORESOURCE_IO)
			mask |= MASK_SPC;

		pci_write_reg(pp, mask, RCAR_PCIEPTCTLR(win));

		win++;
	}

	return 1;

out:
	printk("pcie: request_resource failed\n");
	for (--i; i >= 0; i--)
		release_resource(&rcar_pci0_resources[i]);

	return 0;
}

static int rcar_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return rcar_pcie.irq;
}

static struct pci_bus __init *rcar_pcie_scan_bus(int nr,
						  struct pci_sys_data *sys)
{
	struct rcar_pcie_port *pp;

	if (nr >= rcar_pcie.num_ports)
		return NULL;

	pp = rcar_pcie.port + nr;
	pp->root_bus_nr = sys->busnr;

	return pci_scan_root_bus(NULL, sys->busnr, &rcar_pcie_ops, sys,
				 &sys->resources);
}

static struct hw_pci rcar_pcie_hw __initdata = {
	.nr_controllers	= 1,
	.setup		= rcar_pcie_setup,
	.scan		= rcar_pcie_scan_bus,
	.swizzle	= pci_std_swizzle,
	.map_irq	= rcar_pcie_map_irq,
};


static int __init phy_wait_for_ack(struct rcar_pcie_port *chan)
{
	unsigned int timeout = 100;

	while (timeout--) {
		if (pci_read_reg(chan, RCAR_PCIEPHYADRR) & (1 << BITS_ACK))
			return 0;

		udelay(100);
	}

	printk(KERN_ERR "%s: Access to PCIe phy timed out!", __func__);

	return -ETIMEDOUT;
}

static void __init phy_write_reg(struct rcar_pcie_port *chan, unsigned int rate, unsigned int addr,
				 unsigned int lane, unsigned int data)
{
	unsigned long phyaddr;

	phyaddr = (1 << BITS_CMD) + ((rate & 1) << BITS_RATE) +
			((lane & 0xf) << BITS_LANE) +
			((addr & 0xff) << BITS_ADR);

	/* Set write data */
	pci_write_reg(chan, data, RCAR_PCIEPHYDOUTR);
	pci_write_reg(chan, phyaddr, RCAR_PCIEPHYADRR);

	phy_wait_for_ack(chan);

	/* Clear command */
	pci_write_reg(chan, 0, RCAR_PCIEPHYDOUTR);
	pci_write_reg(chan, 0, RCAR_PCIEPHYADRR);

	phy_wait_for_ack(chan);
}

static int __init phy_init(struct rcar_pcie_port *chan)
{
	unsigned int timeout = 100;

	/* Initialize the phy */
	phy_write_reg(chan, 0, 0x42, 0x1, 0x0EC34191);
	phy_write_reg(chan, 1, 0x42, 0x1, 0x0EC34180);
	phy_write_reg(chan, 0, 0x43, 0x1, 0x00210188);
	phy_write_reg(chan, 1, 0x43, 0x1, 0x00210188);
	phy_write_reg(chan, 0, 0x44, 0x1, 0x015C0014);
	phy_write_reg(chan, 1, 0x44, 0x1, 0x015C0014);
	phy_write_reg(chan, 1, 0x4C, 0x1, 0x786174A0);
	phy_write_reg(chan, 1, 0x4D, 0x1, 0x048000BB);
	phy_write_reg(chan, 0, 0x51, 0x1, 0x079EC062);
	phy_write_reg(chan, 0, 0x52, 0x1, 0x20000000);
	phy_write_reg(chan, 1, 0x52, 0x1, 0x20000000);
	phy_write_reg(chan, 1, 0x56, 0x1, 0x00003806);

	phy_write_reg(chan, 0, 0x60, 0x1, 0x004B03A5);
	phy_write_reg(chan, 0, 0x64, 0x1, 0x3F0F1F0F);
	phy_write_reg(chan, 0, 0x66, 0x1, 0x00008000);

	while (timeout--) {
		if (pci_read_reg(chan, RCAR_PCIEPHYSR))
			return 0;

		udelay(100);
	}

	return -ETIMEDOUT;
}

static int __init pci_wait_for_dl(struct rcar_pcie_port *chan)
{
	unsigned int timeout = 100;

	while (timeout--) {
		if ((pci_read_reg(chan, RCAR_PCIETSTR) & MASK_INT_DLLACT))
			return 0;

		udelay(100);
	}

	return -ETIMEDOUT;
}

static void __init pcie_reset(struct rcar_pcie_port *chan)
{
	pci_write_reg(chan, 0, RCAR_PCIETCTLR);
}

static int __init rcar_pcie_hw_init(struct rcar_pcie_port *chan)
{
	unsigned int data;
	phys_addr_t memphys;
	size_t memsize;
	int ret;

	/* Begin initialization */
	pcie_reset(chan);

	/*
	 * Initial header for port config space is type 1, set the device
	 * class to match. Hardware takes care of propagating the IDSETR
	 * settings, so there is no need to bother with a quirk.
	 */
	pci_write_reg(chan, PCI_CLASS_BRIDGE_PCI << 16, RCAR_PCIEIDSETR1);

	/* Initialize default capabilities. */
	data = pci_read_reg(chan, RCAR_PCIEEXPCAP(0));
	data &= ~(PCI_EXP_FLAGS_TYPE << 16);

	if (chan->endpoint)
		data |= PCI_EXP_TYPE_ENDPOINT << 20;
	else
		data |= PCI_EXP_TYPE_ROOT_PORT << 20;

	data |= PCI_CAP_ID_EXP;
	pci_write_reg(chan, data, RCAR_PCIEEXPCAP(0));

	data = pci_read_reg(chan, RCAR_PCIEPCICONF(3));
	if (chan->endpoint)
		data |= 1 << 16;
	pci_write_reg(chan, data, RCAR_PCIEPCICONF(3));

	/* Enable data link layer active state reporting */
	data = pci_read_reg(chan, RCAR_PCIEEXPCAP(3));
	data |= PCI_EXP_LNKCAP_DLLLARC;
	pci_write_reg(chan, data, RCAR_PCIEEXPCAP(3));

	/* Enable extended sync and ASPM L0s support */
	data = pci_read_reg(chan, RCAR_PCIEEXPCAP(4));
	data &= ~PCI_EXP_LNKCTL_ASPMC;
	data |= PCI_EXP_LNKCTL_ES | 1;
	pci_write_reg(chan, data, RCAR_PCIEEXPCAP(4));

	/* Write out the physical slot number */
	data = pci_read_reg(chan, RCAR_PCIEEXPCAP(5));
	data &= ~PCI_EXP_SLTCAP_PSN;
	data |= (chan->index + 1) << 19;
	pci_write_reg(chan, data, RCAR_PCIEEXPCAP(5));

	/* Set the completion timer timeout to the maximum 50ms. */
	data = pci_read_reg(chan, RCAR_PCIETLCTLR);
	data &= ~(0x3f << 8);
	data |= 0x32 << 8;
	pci_write_reg(chan, data, RCAR_PCIETLCTLR);

	/* Enable MAC data scrambling. */
	data = pci_read_reg(chan, RCAR_PCIEMACCTLR);
	data &= ~PCIEMACCTLR_SCR_DIS;
	pci_write_reg(chan, data, RCAR_PCIEMACCTLR);

	/* Next Capability Offset */
	data = pci_read_reg(chan, RCAR_PCIEVCCAP(0));
	data &= ~(0xfff << 20);
	data |=  0x1b0 << 20;
	pci_write_reg(chan, data, RCAR_PCIEVCCAP(0));

	memphys = PHYS_OFFSET;
	memsize = MEM_SIZE;

	/* LAR0/LAMR0 covers up to 4GB */
	pci_write_reg(chan, memphys, RCAR_PCIELAR(0));
	pci_write_reg(chan, (memsize - SZ_256) | 1, RCAR_PCIELAMR(0));

	/* Disable MAC data scrambling. */
	data = pci_read_reg(chan, RCAR_PCIEMACCTLR);
	data |= PCIEMACCTLR_SCR_DIS;
	pci_write_reg(chan, data, RCAR_PCIEMACCTLR);

	data = pci_read_reg(chan, RCAR_PCIEPCICONF(1));
	data &= ~(PCI_STATUS_DEVSEL_MASK << 16);
	data |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
		(PCI_STATUS_CAP_LIST | PCI_STATUS_DEVSEL_FAST) << 16;
	pci_write_reg(chan, data, RCAR_PCIEPCICONF(1));

	/* Finish initialization */
	data = pci_read_reg(chan, RCAR_PCIETCTLR);
	data |= 0x1;
	pci_write_reg(chan, data, RCAR_PCIETCTLR);

	/* This will timeout if we don't have a link. */
	ret = pci_wait_for_dl(chan);

	/* Enable INTx interupts */
	data = pci_read_reg(chan, RCAR_PCIEINTXR);
	if (!chan->endpoint)
		data |= 0x1 << 8;
	pci_write_reg(chan, data, RCAR_PCIEINTXR);

	data = pci_read_reg(chan, RCAR_PCIEPCICONF(15));
	data &= ~0xFFFF;
	data |= 0x100;
	pci_write_reg(chan, data, RCAR_PCIEPCICONF(15));

	wmb();

	if (ret == 0) {
		data = pci_read_reg(chan, RCAR_PCIEMACSR);
		printk(KERN_NOTICE "PCI: PCIe#%d x%d link detected\n",
		       chan->index, (data >> 20) & 0x3f);
	} else {
		printk(KERN_NOTICE "PCI: PCIe#%d link down\n",
		       chan->index);
	}

	return ret;
}

static int rcar_pcie_clocks_get(struct platform_device *pdev)
{
	int err = 0;

	rcar_pcie.pciec_clk = clk_get(&pdev->dev, "pciec");
	if (IS_ERR(rcar_pcie.pciec_clk))
		err = PTR_ERR(rcar_pcie.pciec_clk);
	else
		clk_enable(rcar_pcie.pciec_clk);

	return err;
}

static void rcar_pcie_clocks_put(void)
{
	clk_put(rcar_pcie.pciec_clk);
}

static int __init rcar_pcie_get_resources(struct platform_device *pdev)
{
	struct resource *res;
	int i;
	int err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i = platform_get_irq(pdev, 0);
	if (!res || i < 0) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		return -ENOENT;
	}
	rcar_pcie.irq = i;

	err = rcar_pcie_clocks_get(pdev);
	if (err) {
		pr_err("PCIE: failed to get clocks: %d\n", err);
		return err;
	}

	rcar_pcie.regs = ioremap_nocache(res->start, resource_size(res));
	if (rcar_pcie.regs == NULL) {
		pr_err("PCIE: Failed to map PCIEC registers\n");
		err = -ENOMEM;
		goto err_map_reg;
	}

	return 0;

err_map_reg:
	rcar_pcie_clocks_put();

	return err;
}

static int __init rcar_pcie_add_port(int index, u32 offset)
{
	struct rcar_pcie_port *pp;

	pp = rcar_pcie.port + rcar_pcie.num_ports;

	pp->endpoint = 0;
	pp->index = -1;
	pp->base = rcar_pcie.regs + offset;

	pci_write_reg(pp, pp->endpoint, RCAR_PCIEMSR);

	if (phy_init(&rcar_pcie.port[index])) {
		printk(KERN_INFO "PCIe: port %d: phy init failed\n", index);
		return -1;
	}

	rcar_pcie.num_ports++;
	pp->index = index;
	pp->root_bus_nr = -1;

	if (rcar_pcie_hw_init(&rcar_pcie.port[index]))
		return -1;

	return 0;
}

int __init rcar_pcie_init(struct platform_device *pdev)
{
	int err;

	err = rcar_pcie_get_resources(pdev);
	if (err)
		return err;

	err = rcar_pcie_add_port(0, 0);
	if (err)
		return err;

	pci_common_init(&rcar_pcie_hw);

	return 0;
}
