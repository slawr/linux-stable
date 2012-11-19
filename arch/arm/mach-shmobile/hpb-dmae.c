/*
 * arch/arm/mach-rcar/hpb-dmae.c
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/rcar-hpbdma.h>
#include <mach/hardware.h>
#include <mach/hpb-dmae.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

/* Transmit sizes and respective register values */
enum {
	XMIT_SZ_8BIT		= 0,
	XMIT_SZ_16BIT		= 1,
	XMIT_SZ_32BIT		= 2,
};

/* log2(size / 8) - used to calculate number of transfers */
#define TS_SHIFT {			\
	[XMIT_SZ_8BIT]		= 0,	\
	[XMIT_SZ_16BIT]		= 1,	\
	[XMIT_SZ_32BIT]		= 2,	\
}

static const struct hpb_dmae_slave_config rcar_dmae_slaves[] = {
	{
		.id	= HPBDMA_SLAVE_SDHI0_TX,
		.addr	= 0xffe4c000 + 0x30,
		.dcr	= SPDS_16BIT | DMDL | DPDS_16BIT,
		.rstr	= ASRST21 | ASRST22 | ASRST23,
		.mdr	= ASMD21_SINGLE | ASBTMD21_NBURST,
		.port	= 0x0D0C,
		.flags	= HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE,
		.dma_ch	= 21,
	}, {
		.id	= HPBDMA_SLAVE_SDHI0_RX,
		.addr	= 0xffe4c000 + 0x30,
		.dcr	= SMDL | SPDS_16BIT | DPDS_16BIT,
		.rstr	= ASRST21 | ASRST22 | ASRST23,
		.mdr	= ASMD22_SINGLE | ASBTMD22_NBURST,
		.port	= 0x0D0C,
		.flags	= HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE |
				HPB_DMAE_SET_SHPT1,
		.dma_ch	= 22,
	}, {
		.id	= HPBDMA_SLAVE_SSI0_TX_ST,
		.addr	= 0xffd90000 + 0x1008,
		.dcr	= CT | DIP | SPDS_32BIT | DMDL | DPDS_32BIT,
		.port	= 0x0000,
		.flags	= 0,
		.dma_ch	= 28,
	}, {
		.id	= HPBDMA_SLAVE_SSI1_RX_ST,
		.addr	= 0xffd90000 + 0x104c,
		.dcr	= CT | DIP | SMDL | SPDS_32BIT | DPDAM | DPDS_32BIT,
		.port	= 0x0101,
		.flags	= 0,
		.dma_ch	= 29,
	},
};

#define DMAE_CHANNEL(_offset, _irq, _s_id)	\
	{						\
		.offset		= _offset,		\
		.ch_irq		= _irq,			\
		.s_id		= _s_id,			\
	}

/* comment out for not using Ch */
static const struct hpb_dmae_channel rcar_dmae_channels[] = {
	/* ch.21 SD0 */
	DMAE_CHANNEL(0x540, IRQ_DMAC_H(21), HPBDMA_SLAVE_SDHI0_TX),
	/* ch.22 SD0 */
	DMAE_CHANNEL(0x580, IRQ_DMAC_H(22), HPBDMA_SLAVE_SDHI0_RX),
	/* ch.28 SSI0 */
	DMAE_CHANNEL(0x700, IRQ_DMAC_H(28), HPBDMA_SLAVE_SSI0_TX_ST),
	DMAE_CHANNEL(0x700, IRQ_DMAC_H(28), HPBDMA_SLAVE_SSI0_TX_MN),
	/* ch.29 SSI1 */
	DMAE_CHANNEL(0x740, IRQ_DMAC_H(29), HPBDMA_SLAVE_SSI1_RX_ST),
	DMAE_CHANNEL(0x740, IRQ_DMAC_H(29), HPBDMA_SLAVE_SSI1_RX_MN),
};

static const unsigned int ts_shift[] = TS_SHIFT;

static struct hpb_dmae_pdata dma_platform_data = {
	.slave		= rcar_dmae_slaves,
	.slave_num	= ARRAY_SIZE(rcar_dmae_slaves),
	.channel	= rcar_dmae_channels,
	.channel_num	= ARRAY_SIZE(rcar_dmae_channels),
	.ts_shift	= ts_shift,
	.ts_shift_num	= ARRAY_SIZE(ts_shift),
};

/* Resource order important! */
static struct resource rcar_dmae_resources[] = {
	{
		/* Channel registers */
		.start	= 0xffc08000,
		.end	= 0xffc08000 + (0x40 * HPB_DMA_MAXCH) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Common registers */
		.start	= 0xffc09000,
		.end	= 0xffc09170 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Asynchronous reset registers */
		.start	= 0xffc00300,
		.end	= 0xffc00304 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Asynchronous mode registers */
		.start	= 0xffc00400,
		.end	= 0xffc00404 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* IRQ for channels DMA ch.20 - ch.43 */
		.start	= IRQ_DMAC_H(HPB_DMA_USE_START_CH),
		.end	= IRQ_DMAC_H(HPB_DMA_USE_END_CH),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device rcar_dma_device = {
	.name		= "hpb-dma-engine",
	.id		= 0,
	.resource	= rcar_dmae_resources,
	.num_resources	= ARRAY_SIZE(rcar_dmae_resources),
	.dev		= {
		.platform_data	= &dma_platform_data,
	},
};
