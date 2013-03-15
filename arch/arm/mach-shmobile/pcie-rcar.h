/*
 * PCI Express definitions for r8a7779 (R-Car H1) SoC
 *  Copyright (C) 2013 Renesas Electronics Europe Ltd
 *
 * Based on arch/sh/drivers/pci/pcie-sh7786.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __PCI_RCAR_H
#define __PCI_RCAR_H

#define	RCAR_PCIEPAR		(0x000010)
#define		BITS_BN		(24)
#define		MASK_BN		(0xff<<BITS_BN)
#define		BITS_DN		(19)
#define		MASK_DN		(0x1f<<BITS_DN)
#define		BITS_FN		(16)
#define		MASK_FN		(0x7<<BITS_FN)
#define		BITS_EREGNO	(8)
#define		MASK_EREGNO	(0xff<<BITS_EREGNO)
#define		BITS_REGNO	(2)
#define		MASK_REGNO	(0x3f<<BITS_REGNO)

#define	RCAR_PCIEPCTLR		(0x000018)
#define		BITS_CCIE	(31)	/*  31 CCIE */
#define		MASK_CCIE	(1<<BITS_CCIE)
#define		BITS_TYPE	(8)
#define		MASK_TYPE	(1<<BITS_TYPE)
#define		BITS_C_VC	(0)
#define		MASK_C_VC	(1<<BITS_C_VC)

#define	RCAR_PCIEPDR		(0x000020)
#define	RCAR_PCIEMSR		(0x000028)
#define	RCAR_PCIEINTXR		(0x000400)

/* Transfer control */
#define	RCAR_PCIETCTLR		(0x02000)
#define		BITS_CFINT	(0)
#define		MASK_CFINT	(1<<BITS_CFINT)
#define	RCAR_PCIETSTR		(0x02004)
#define		BITS_INT_DLLACT			(0)
#define		MASK_INT_DLLACT			(1 << BITS_INT_DLLACT)
#define	RCAR_PCIEINTR		(0x02008)
#define	RCAR_PCIEINTER		(0x0200C)
#define		BITS_INT_RX_ERP			(31)
#define		MASK_INT_RX_ERP			(1<<BITS_INT_RX_ERP)
#define		BITS_INT_RX_VCX_Posted		(30)
#define		MASK_INT_RX_VCX_Posted		(1<<BITS_INT_RX_VCX_Posted)
#define		BITS_INT_RX_VCX_NonPosted	(29)
#define		MASK_INT_RX_VCX_NonPosted	(1<<BITS_INT_RX_VCX_NonPosted)
#define		BITS_INT_RX_VCX_CPL		(28)
#define		MASK_INT_RX_VCX_CPL		(1<<BITS_INT_RX_VCX_CPL)
#define		BITS_INT_TX_VCX_Posted		(26)
#define		MASK_INT_TX_VCX_Posted		(1<<BITS_INT_TX_VCX_Posted)
#define		BITS_INT_TX_VCX_NonPosted	(25)
#define		MASK_INT_TX_VCX_NonPosted	(1<<BITS_INT_TX_VCX_NonPosted)
#define		BITS_INT_TX_VCX_CPL		(24)
#define		MASK_INT_TX_VCX_CPL		(1<<BITS_INT_TX_VCX_CPL)
#define		BITS_INT_RX_VC0_Posted		(22)
#define		MASK_INT_RX_VC0_Posted		(1<<BITS_INT_RX_VC0_Posted)
#define		BITS_INT_RX_VC0_NonPosted	(21)
#define		MASK_INT_RX_VC0_NonPosted	(1<<BITS_INT_RX_VC0_NonPosted)
#define		BITS_INT_RX_VC0_CPL		(20)
#define		MASK_INT_RX_VC0_CPL		(1<<BITS_INT_RX_VC0_CPL)
#define		BITS_INT_TX_VC0_Posted		(18)
#define		MASK_INT_TX_VC0_Posted		(1<<BITS_INT_TX_VC0_Posted)
#define		BITS_INT_TX_VC0_NonPosted	(17)
#define		MASK_INT_TX_VC0_NonPosted	(1<<BITS_INT_TX_VC0_NonPosted)
#define		BITS_INT_TX_VC0_CPL		(16)
#define		MASK_INT_TX_VC0_CPL		(1<<BITS_INT_TX_VC0_CPL)
#define		BITS_INT_RX_CTRL		(15)
#define		MASK_INT_RX_CTRL		(1<<BITS_INT_RX_CTRL)
#define		BITS_INT_TX_CTRL		(14)
#define		MASK_INT_TX_CTRL		(1<<BITS_INT_TX_CTRL)
#define		BITS_INTTL			(11)
#define		MASK_INTTL			(1<<BITS_INTTL)
#define		BITS_INTDL			(10)
#define		MASK_INTDL			(1<<BITS_INTDL)
#define		BITS_INTMAC			(9)
#define		MASK_INTMAC			(1<<BITS_INTMAC)
#define		BITS_INTPM			(8)
#define		MASK_INTPM			(1<<BITS_INTPM)
#define		BITS_INT_MESE			(5)
#define		MASK_INT_MESE			(1<<BITS_INT_MESE)
#define	RCAR_PCIEERRFR		(0x02020)
#define	RCAR_PCIEERRFER		(0x02024)

/* root port address */
#define	RCAR_PCIEPRAR(x)	(0x02080 + ((x) * 0x4))

/* local address reg & mask */
#define	RCAR_PCIELAR(x)		(0x02200 + ((x) * 0x20))
#define	RCAR_PCIELAMR(x)	(0x02208 + ((x) * 0x20))

/* PCIe address reg & mask */
#define	RCAR_PCIEPARL(x)	(0x03400 + ((x) * 0x20))
#define	RCAR_PCIEPARH(x)	(0x03404 + ((x) * 0x20))
#define	RCAR_PCIEPAMR(x)	(0x03408 + ((x) * 0x20))
#define	RCAR_PCIEPTCTLR(x)	(0x0340C + ((x) * 0x20))
#define		BITS_PARE	(31)
#define		MASK_PARE	(0x1<<BITS_PARE)
#define		BITS_TC		(20)
#define		MASK_TC		(0x7<<BITS_TC)
#define		BITS_T_VC	(16)
#define		MASK_T_VC	(0x1<<BITS_T_VC)
#define		BITS_LOCK	(12)
#define		MASK_LOCK	(0x1<<BITS_LOCK)
#define		BITS_SPC	(8)
#define		MASK_SPC	(0x1<<BITS_SPC)

/* DMA */
#define	RCAR_PCIEDMAOR		(0x04000)
#define	RCAR_PCIEDMPALR(x)	(0x04100 + ((x) * 0x40))
#define	RCAR_PCIEDMPAUR(x)	(0x04104 + ((x) * 0x40))
#define	RCAR_PCIEDMIAR(x)	(0x04108 + ((x) * 0x40))
#define	RCAR_PCIEDMBCNTR(x)	(0x04110 + ((x) * 0x40))
#define	RCAR_PCIEDMCCAR(x)	(0x04120 + ((x) * 0x40))
#define	RCAR_PCIEDMCHCR(x)	(0x04128 + ((x) * 0x40))
#define	RCAR_PCIEDMCHSR(x)	(0x0412C + ((x) * 0x40))
#define	RCAR_PCIEDMCHC2R(x)	(0x04130 + ((x) * 0x40))

/* Configuration */
#define	RCAR_PCIEPCICONF(x)	(0x010000 + ((x) * 0x4))

#define	RCAR_PCIEPMCAP0		(0x010040)
#define	RCAR_PCIEPMCAP1		(0x010044)
#define	RCAR_PCIEMSICAP(x)	(0x010050 + ((x) * 0x4))
#define	RCAR_PCIEEXPCAP(x)	(0x010070 + ((x) * 0x4))
#define	RCAR_PCIEVCCAP(x)	(0x010100 + ((x) * 0x4))

#define	RCAR_PCIENUMCAP0	(0x0101B0)
#define	RCAR_PCIENUMCAP1	(0x0101B4)
#define	RCAR_PCIENUMCAP2	(0x0101B8)

#define	RCAR_PCIEIDSETR0	(0x011000)
#define	RCAR_PCIEIDSETR1	(0x011004)
#define	RCAR_PCIEDSERSETR0	(0x01102C)
#define	RCAR_PCIEDSERSETR1	(0x011030)
#define	RCAR_PCIETLCTLR		(0x011048)
#define	RCAR_PCIEMACSR		(0x011054)
#define	RCAR_PCIEMACCTLR	(0x011058)
#define		PCIEMACCTLR_SCR_DIS	(1 << 27)	/* scramble disable */
#define	RCAR_PCIEPMSTR		(0x01105C)
#define	RCAR_PCIEPMCTLR		(0x011060)
#define	RCAR_PCIEMACINTENR	(0x01106C)
#define	RCAR_PCIEPMINTENR	(0x011070)

/* PHY */
#define	RCAR_PCIEPHYCTLR	(0x040008)
#define	RCAR_PCIEPHYADRR	(0x04000C)
#define		BITS_ACK	(24)
#define		MASK_ACK	(1<<BITS_ACK)
#define		BITS_CMD	(16)
#define		MASK_CMD	(0x03<<BITS_CMD)
#define		BITS_RATE	(12)
#define		MASK_RATE	(0x01<<BITS_RATE)
#define		BITS_LANE	(8)
#define		MASK_LANE	(0x0f<<BITS_LANE)
#define		BITS_ADR	(0)
#define		MASK_ADR	(0xff<<BITS_ADR)
#define	RCAR_PCIEPHYDOUTR	(0x040014)
#define	RCAR_PCIEPHYSR		(0x040018)

#endif /* __PCI_RCAR_H */
