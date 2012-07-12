/*
 * marzen board support
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/sh_hspi.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/mfd/tmio.h>
#include <linux/usb/rcar-usb.h>
#include <mach/hardware.h>
#include <mach/r8a7779.h>
#include <mach/common.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/traps.h>

/* Fixed 3.3V regulator to be used by SDHI0 */
static struct regulator_consumer_supply fixed3v3_power_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sh_mobile_sdhi.0"),
	REGULATOR_SUPPLY("vqmmc", "sh_mobile_sdhi.0"),
};

/* Dummy supplies, where voltage doesn't matter */
static struct regulator_consumer_supply dummy_supplies[] = {
	REGULATOR_SUPPLY("vddvario", "smsc911x"),
	REGULATOR_SUPPLY("vdd33a", "smsc911x"),
};

/* SMSC LAN89218 */
static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x18000000, /* ExCS0 */
		.end		= 0x180000ff, /* A1->A7 */
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= gic_spi(28), /* IRQ 1 */
		.flags		= IORESOURCE_IRQ,
	},
};

static struct smsc911x_platform_config smsc911x_platdata = {
	.flags		= SMSC911X_USE_32BIT, /* 32-bit SW on 16-bit HW bus */
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
};

static struct platform_device eth_device = {
	.name		= "smsc911x",
	.id		= -1,
	.dev  = {
		.platform_data = &smsc911x_platdata,
	},
	.resource	= smsc911x_resources,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
};

static struct resource sdhi0_resources[] = {
	[0] = {
		.name	= "sdhi0",
		.start	= 0xffe4c000,
		.end	= 0xffe4c0ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= gic_spi(104),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct sh_mobile_sdhi_info sdhi0_platform_data = {
	.tmio_flags = TMIO_MMC_WRPROTECT_DISABLE | TMIO_MMC_HAS_IDLE_WAIT,
	.tmio_caps = MMC_CAP_SD_HIGHSPEED,
};

static struct platform_device sdhi0_device = {
	.name = "sh_mobile_sdhi",
	.num_resources = ARRAY_SIZE(sdhi0_resources),
	.resource = sdhi0_resources,
	.id = 0,
	.dev = {
		.platform_data = &sdhi0_platform_data,
	}
};

/* Thermal */
static struct resource thermal_resources[] = {
	[0] = {
		.start		= 0xFFC48000,
		.end		= 0xFFC48038 - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device thermal_device = {
	.name		= "rcar_thermal",
	.resource	= thermal_resources,
	.num_resources	= ARRAY_SIZE(thermal_resources),
};

/* HSPI */
static struct resource hspi_resources[] = {
	[0] = {
		.start		= 0xFFFC7000,
		.end		= 0xFFFC7018 - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device hspi_device = {
	.name	= "sh-hspi",
	.id	= 0,
	.resource	= hspi_resources,
	.num_resources	= ARRAY_SIZE(hspi_resources),
};

static u64 usb_dmamask = ~(u32)0;

static struct resource ehci0_resources[] = {
	[0] = {
		.start	= 0xffe70000,
		.end	= 0xffe70000 + 0x3ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USBH_0_EHCI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rcar_usb_info ehci0_info = {
	.port_offset = 0,
};

struct platform_device ehci0_device = {
	.name	= "rcar_ehci",
	.id	= 0,
	.dev	= {
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ehci0_info,
	},
	.num_resources	= ARRAY_SIZE(ehci0_resources),
	.resource	= ehci0_resources,
};

static struct resource ohci0_resources[] = {
	[0] = {
		.start	= 0xffe70400,
		.end	= 0xffe70400 + 0x3ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USBH_0_OHCI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rcar_usb_info ohci0_info = {
	.port_offset = 0,
};

struct platform_device ohci0_device = {
	.name	= "rcar_ohci",
	.id	= 0,
	.dev	= {
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ohci0_info,
	},
	.num_resources	= ARRAY_SIZE(ohci0_resources),
	.resource	= ohci0_resources,
};

static struct resource ehci1_resources[] = {
	[0] = {
		.start	= 0xfff70000,
		.end	= 0xfff70000 + 0x3ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USBH_1_EHCI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rcar_usb_info ehci1_info = {
	.port_offset = 1,
};

struct platform_device ehci1_device = {
	.name	= "rcar_ehci",
	.id	= 1,
	.dev	= {
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ehci1_info,
	},
	.num_resources	= ARRAY_SIZE(ehci1_resources),
	.resource	= ehci1_resources,
};

static struct resource ohci1_resources[] = {
	[0] = {
		.start	= 0xfff70400,
		.end	= 0xfff70400 + 0x3ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USBH_1_OHCI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rcar_usb_info ohci1_info = {
	.port_offset = 1,
};

struct platform_device ohci1_device = {
	.name	= "rcar_ohci",
	.id	= 1,
	.dev	= {
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ohci1_info,
	},
	.num_resources	= ARRAY_SIZE(ohci1_resources),
	.resource	= ohci1_resources,
};

#define RC_USBH0_BASE 0xffe70000
#define RC_USBH1_BASE 0xfff70000

/* USBH common register */
#define USBPCTRL0		0x0800
#define USBPCTRL1		0x0804
#define USBST			0x0808
#define USBEH0			0x080C
#define USBOH0			0x081C
#define USBCTL0			0x0858
#define EIIBC1			0x0094
#define EIIBC2			0x009C

/* bit field of Port Control 0 register */
#define	USBPCTRL0_PORT1		0x00000001
#define	USBPCTRL0_OVC1		0x00000002
#define	USBPCTRL0_OVC0		0x00000008
#define	USBPCTRL0_PENC		0x00000010
#define	USBPCTRL0_PORT0		0x00000100
#define	USBPCTRL0_OVC1_VBUS1	0x00000200

/* bit field of Port Control 1 register */
#define	USBPCTRL1_PHYENB	0x00000001
#define	USBPCTRL1_PLLENB	0x00000002
#define	USBPCTRL1_PHYRST	0x00000004
#define	USBPCTRL1_RST		0x80000000

/* bit field of USB Status register */
#define	USBST_PLL		0x40000000
#define	USBST_ACT		0x80000000

/* bit field of USB Control0 register */
#define	USBCTL0_CLKSEL		0x00000080

/* bit field of EHCI IP Buffer Control register 2 */
#define	EIIBC2_BUF_EN		0x00000001

/* other */
#define EHCI_IP_BUF		0x00FF0040
#define SWAP_NONE		0x00000000
#define SWAP_BYTE		0x00000001
#define SWAP_WORD		0x00000002
#define SWAP_WORD_BYTE		0x00000003

static int __init rcar_usbh_start(void)
{
	u32 val;
	int timeout;
	struct clk *clk, *clk2;

	void __iomem *usb_base = ioremap_nocache(RC_USBH0_BASE, 0x900);
	void __iomem *usb_base1 = ioremap_nocache(RC_USBH1_BASE, 0x900);

	/* enable clocks USB0/1 and 2 */
	clk = clk_get(NULL, "usb_fck");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "Can't get usb clock\n");
		return -1;
	}
	clk_enable(clk);

	clk2 = clk_get(NULL, "usb_fck2");
	if (IS_ERR(clk2)) {
		printk(KERN_ERR "Can't get usb clock 2\n");
		return -1;
	}
	clk_enable(clk2);

	/*
	 * Port Control 1 Setting
	 */
	/* clear stand-by */
	iowrite32(USBPCTRL1_PHYENB, (usb_base + USBPCTRL1));

	/* start PLL */
	iowrite32((USBPCTRL1_PLLENB | USBPCTRL1_PHYENB),
		(usb_base + USBPCTRL1));

	/* check status */
	timeout = 100; /* about 100ms */
	do {
		msleep(1);
		val = ioread32(usb_base + USBST);
		if ((val & USBST_ACT) && (val & USBST_PLL))
			break;
	} while (--timeout > 0);
	if (timeout == 0) {
		printk(KERN_ERR "USB is not ready. [%08x]\n", val);
		return -1;
	}

	/* clear reset */
	iowrite32((USBPCTRL1_PHYRST | USBPCTRL1_PLLENB | USBPCTRL1_PHYENB),
		(usb_base + USBPCTRL1));

	/*
	 * Port Control 0 Setting
	 */
	iowrite32(0, (usb_base + USBPCTRL0));

	/*
	 * EHCI IP Internal Buffer Setting
	 */
	iowrite32(EHCI_IP_BUF, (usb_base + EIIBC1));
	iowrite32(EIIBC2_BUF_EN, (usb_base + EIIBC2));

	iowrite32(EHCI_IP_BUF, (usb_base1 + EIIBC1));
	iowrite32(EIIBC2_BUF_EN, (usb_base1 + EIIBC2));

	/*
	 * Bus Alignment Setting
	 */
	iowrite32(SWAP_NONE, (usb_base + USBEH0));
	iowrite32(SWAP_NONE, (usb_base + USBOH0));

	iounmap(usb_base);
	iounmap(usb_base1);

	return 0;
}

static int __init rcar_usbh_init(void)
{
	if (rcar_usbh_start()) {
		platform_device_unregister(&ehci0_device);
		platform_device_unregister(&ohci0_device);
		platform_device_unregister(&ehci1_device);
		platform_device_unregister(&ohci1_device);
	}

	return 0;
}

static struct platform_device *marzen_devices[] __initdata = {
	&eth_device,
	&sdhi0_device,
	&thermal_device,
	&hspi_device,
	&ehci0_device,
	&ohci0_device,
	&ehci1_device,
	&ohci1_device,
};

static void __init marzen_init(void)
{
	regulator_register_fixed(0, fixed3v3_power_consumers,
				ARRAY_SIZE(fixed3v3_power_consumers));
	regulator_register_fixed(0, dummy_supplies,
				ARRAY_SIZE(dummy_supplies));

	r8a7779_pinmux_init();

	/* SCIF2 (CN18: DEBUG0) */
	gpio_request(GPIO_FN_TX2_C, NULL);
	gpio_request(GPIO_FN_RX2_C, NULL);

	/* SCIF4 (CN19: DEBUG1) */
	gpio_request(GPIO_FN_TX4, NULL);
	gpio_request(GPIO_FN_RX4, NULL);

	/* LAN89218 */
	gpio_request(GPIO_FN_EX_CS0, NULL); /* nCS */
	gpio_request(GPIO_FN_IRQ1_B, NULL); /* IRQ + PME */

	/* SD0 (CN20) */
	gpio_request(GPIO_FN_SD0_CLK, NULL);
	gpio_request(GPIO_FN_SD0_CMD, NULL);
	gpio_request(GPIO_FN_SD0_DAT0, NULL);
	gpio_request(GPIO_FN_SD0_DAT1, NULL);
	gpio_request(GPIO_FN_SD0_DAT2, NULL);
	gpio_request(GPIO_FN_SD0_DAT3, NULL);
	gpio_request(GPIO_FN_SD0_CD, NULL);
	gpio_request(GPIO_FN_SD0_WP, NULL);

	/* HSPI 0 */
	gpio_request(GPIO_FN_HSPI_CLK0,	NULL);
	gpio_request(GPIO_FN_HSPI_CS0,	NULL);
	gpio_request(GPIO_FN_HSPI_TX0,	NULL);
	gpio_request(GPIO_FN_HSPI_RX0,	NULL);

	/* USB (CN21) */
	gpio_request(GPIO_FN_USB_OVC0, NULL);
	gpio_request(GPIO_FN_USB_OVC1, NULL);
	gpio_request(GPIO_FN_USB_OVC2, NULL);

	r8a7779_add_standard_devices();
	platform_add_devices(marzen_devices, ARRAY_SIZE(marzen_devices));

	rcar_usbh_init();
}

MACHINE_START(MARZEN, "marzen")
	.map_io		= r8a7779_map_io,
	.init_early	= r8a7779_add_early_devices,
	.nr_irqs	= NR_IRQS_LEGACY,
	.init_irq	= r8a7779_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= marzen_init,
	.timer		= &shmobile_timer,
MACHINE_END
