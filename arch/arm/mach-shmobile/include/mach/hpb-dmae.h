/*
 * arch/arm/mach-rcar/include/mach/hpb-dmae.h
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

#ifndef	__ARCH_MACH_HPBDMAE_H
#define	__ARCH_MACH_HPBDMAE_H

extern struct platform_device rcar_dma_device;

/* DMA slave IDs */
enum {
	HPBDMA_SLAVE_INVALID,		/* DMA slave ID 0 means no DMA */
	HPBDMA_SLAVE_SDHI0_TX,		/*Ch.21*/
	HPBDMA_SLAVE_SDHI0_RX,		/*Ch.22*/
	HPBDMA_SLAVE_MMC0_TX,		/*Ch.24*/
	HPBDMA_SLAVE_MMC0_RX,		/*Ch.24*/
	HPBDMA_SLAVE_SSI0_TX_ST,	/*Ch.28*/
	HPBDMA_SLAVE_SSI1_RX_ST,	/*Ch.29*/
	HPBDMA_SLAVE_SSI7_TX_ST,	/*Ch.35*/
	HPBDMA_SLAVE_SSI8_RX_ST,	/*Ch.36*/
	HPBDMA_SLAVE_MMC1_TX,		/*Ch.43*/
	HPBDMA_SLAVE_MMC1_RX,		/*Ch.43*/
};

#endif /* __ARCH_MACH_HPBDMAE_H */
