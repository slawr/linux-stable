/*
 *  sound/soc/rcar/sru_pcm.h
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

#ifndef __SRU_PCM_H__
#define __SRU_PCM_H__

#include <linux/rcar-hpbdma.h>
#include <mach/hpb-dmae.h>

/************************************************************************

	define

************************************************************************/
/* buffer information */
#define PERIOD_BYTES_MIN	(4 * 1024)
#define PERIOD_BYTES_MAX	(4 * 1024)
#define PERIODS_MIN		16
#define PERIODS_MAX		16
#define BUFFER_BYTES_MAX	(PERIOD_BYTES_MAX * PERIODS_MAX)

/* dma direction */
#define	DMA_DIR(d)	((d == 0) ? DMA_TO_DEVICE : DMA_FROM_DEVICE)

/* DMA information */
#define	DMACH_SSI0	28
#define	DMACH_SSI1	29
#define	DMACH_SSI2	30
#define	DMACH_SSI3	31
#define	DMACH_SSI4	32
#define	DMACH_SSI5	33
#define	DMACH_SSI6	34
#define	DMACH_SSI7	35
#define	DMACH_SSI8	36
#define	DMACH_SSI9	42

#define	CODEC1_PCH	DMACH_SSI0	/* playback */
#define	CODEC1_CCH	DMACH_SSI1	/* capture  */

/* base address */
#define	RC_SRU_BASE	IOMEM(RC_BASE_SRU)		/* 0xffd90000 */
#define	RC_ADG_BASE	IOMEM(RC_BASE_ADG)		/* 0xfffe0000 */

/* HPBIF data register */
#define	HPBIF0_DATA	0xffda0000
#define	HPBIF1_DATA	0xffda1000
#define	HPBIF2_DATA	0xffda2000
#define	HPBIF3_DATA	0xffda3000
#define	HPBIF4_DATA	0xffda4000
#define	HPBIF5_DATA	0xffda5000
#define	HPBIF6_DATA	0xffda6000
#define	HPBIF7_DATA	0xffda7000
#define	HPBIF8_DATA	0xffda8000

/* SRU register offset */
#define	SRC0_BASE		0x00000200
#define	SRC1_BASE		0x00000240
#define	SRC2_BASE		0x00000280
#define	SRC3_BASE		0x000002c0
#define	SRC4_BASE		0x00000300
#define	SRC5_BASE		0x00000340
#define	SRC6_BASE		0x00000380
#define	SRC7_BASE		0x000003c0
#define	SRC8_BASE		0x00000400
#define	CTU00_BASE		0x00000500
#define	CTU01_BASE		0x00000600
#define	CTU02_BASE		0x00000700
#define	CTU03_BASE		0x00000800
#define	CTU10_BASE		0x00000900
#define	CTU11_BASE		0x00000a00
#define	CTU12_BASE		0x00000b00
#define	CTU13_BASE		0x00000c00
#define	MIX0_BASE		0x00000d00
#define	MIX1_BASE		0x00000d40
#define	DVC0_BASE		0x00000e00
#define	DVC1_BASE		0x00000f00
#define	SSI0_BASE		0x00001000
#define	SSI1_BASE		0x00001040
#define	SSI2_BASE		0x00001080
#define	SSI3_BASE		0x000010c0
#define	SSI4_BASE		0x00001100
#define	SSI5_BASE		0x00001140
#define	SSI6_BASE		0x00001180
#define	SSI7_BASE		0x000011c0
#define	SSI8_BASE		0x00001200

/* SRC_ROUTE_SELECT bit */
#define	RTSEL_SRC0_PLAY	(1<<0)	/* HPBIF0 -> SRC0 -> SSI0 */
#define	RTSEL_SRC0_CAPT	(2<<0)	/* SSI0 -> SRC0 -> HPBIF0 */
#define	RTSEL_SRC1_PLAY	(1<<4)	/* HPBIF1 -> SRC1 -> SSI1 */
#define	RTSEL_SRC1_CAPT	(2<<4)	/* SSI1 -> SRC1 -> HPBIF1 */
#define	RTSEL_SRC2_PLAY	(1<<8)	/* HPBIF2 -> SRC2 -> SSI2 */
#define	RTSEL_SRC2_CAPT	(2<<8)	/* SSI2 -> SRC2 -> HPBIF2 */
#define	RTSEL_SRC3_PLAY	(1<<12)	/* HPBIF3 -> SRC3 -> SSI3 */
#define	RTSEL_SRC3_CAPT	(2<<12)	/* SSI3 -> SRC3 -> HPBIF3 */
#define	RTSEL_SRC4_PLAY	(1<<16)	/* HPBIF4 -> SRC4 -> SSI4 */
#define	RTSEL_SRC4_CAPT	(2<<16)	/* SSI4 -> SRC4 -> HPBIF4 */
#define	RTSEL_SRC5_PLAY	(1<<20)	/* HPBIF5 -> SRC5 -> SSI5 */
#define	RTSEL_SRC5_CAPT	(2<<20)	/* SSI5 -> SRC5 -> HPBIF5 */
#define	RTSEL_SRC6_PLAY	(1<<24)	/* HPBIF6 -> SRC6 -> SSI6 */
#define	RTSEL_SRC6_CAPT	(2<<24)	/* SSI6 -> SRC6 -> HPBIF6 */
#define	RTSEL_SRC7_PLAY	(1<<28)	/* HPBIF7 -> SRC7 -> SSI7 */
#define	RTSEL_SRC7_CAPT	(2<<28)	/* SSI7 -> SRC7 -> HPBIF7 */
#define	RTSEL_SRC8_PLAY	(1<<30)	/* HPBIF8 -> SRC8 -> SSI8 */
#define	RTSEL_SRC8_CAPT	(2<<30)	/* SSI8 -> SRC8 -> HPBIF8 */

/* SRC_TIMING_SELECT0 */
#define	TMGSEL0_HPBIF0_ADG	(0<<0)
#define	TMGSEL0_HPBIF0_SSI0_WS	(1<<0)
#define	TMGSEL0_HPBIF0_SSI1_WS	(2<<0)
#define	TMGSEL0_HPBIF0_SSI2_WS	(3<<0)
#define	TMGSEL0_HPBIF0_SSI3_WS	(4<<0)
#define	TMGSEL0_HPBIF0_SSI4_WS	(5<<0)
#define	TMGSEL0_HPBIF0_SSI5_WS	(6<<0)
#define	TMGSEL0_HPBIF0_SSI6_WS	(7<<0)
#define	TMGSEL0_HPBIF0_SSI7_WS	(8<<0)
#define	TMGSEL0_HPBIF0_MIMLCP0	(9<<0)
#define	TMGSEL0_HPBIF0_MIMLCP1	(10<<0)
#define	TMGSEL0_HPBIF0_MIMLCP2	(11<<0)
#define	TMGSEL0_HPBIF0_MIMLCP3	(12<<0)
#define	TMGSEL0_HPBIF0_MIMLCP4	(13<<0)
#define	TMGSEL0_HPBIF0_MIMLCP5	(14<<0)
#define	TMGSEL0_HPBIF0_MIMLCP6	(15<<0)
#define	TMGSEL0_HPBIF1_ADG	(0<<8)
#define	TMGSEL0_HPBIF1_SSI0_WS	(1<<8)
#define	TMGSEL0_HPBIF1_SSI1_WS	(2<<8)
#define	TMGSEL0_HPBIF1_SSI2_WS	(3<<8)
#define	TMGSEL0_HPBIF1_SSI3_WS	(4<<8)
#define	TMGSEL0_HPBIF1_SSI4_WS	(5<<8)
#define	TMGSEL0_HPBIF1_SSI5_WS	(6<<8)
#define	TMGSEL0_HPBIF1_SSI6_WS	(7<<8)
#define	TMGSEL0_HPBIF1_SSI7_WS	(8<<8)
#define	TMGSEL0_HPBIF1_MIMLCP0	(9<<8)
#define	TMGSEL0_HPBIF1_MIMLCP1	(10<<8)
#define	TMGSEL0_HPBIF1_MIMLCP2	(11<<8)
#define	TMGSEL0_HPBIF1_MIMLCP3	(12<<8)
#define	TMGSEL0_HPBIF1_MIMLCP4	(13<<8)
#define	TMGSEL0_HPBIF1_MIMLCP5	(14<<8)
#define	TMGSEL0_HPBIF1_MIMLCP6	(15<<8)
#define	TMGSEL0_HPBIF2_ADG	(0<<16)
#define	TMGSEL0_HPBIF2_SSI0_WS	(1<<16)
#define	TMGSEL0_HPBIF2_SSI1_WS	(2<<16)
#define	TMGSEL0_HPBIF2_SSI2_WS	(3<<16)
#define	TMGSEL0_HPBIF2_SSI3_WS	(4<<16)
#define	TMGSEL0_HPBIF2_SSI4_WS	(5<<16)
#define	TMGSEL0_HPBIF2_SSI5_WS	(6<<16)
#define	TMGSEL0_HPBIF2_SSI6_WS	(7<<16)
#define	TMGSEL0_HPBIF2_SSI7_WS	(8<<16)
#define	TMGSEL0_HPBIF2_MIMLCP0	(9<<16)
#define	TMGSEL0_HPBIF2_MIMLCP1	(10<<16)
#define	TMGSEL0_HPBIF2_MIMLCP2	(11<<16)
#define	TMGSEL0_HPBIF2_MIMLCP3	(12<<16)
#define	TMGSEL0_HPBIF2_MIMLCP4	(13<<16)
#define	TMGSEL0_HPBIF2_MIMLCP5	(14<<16)
#define	TMGSEL0_HPBIF2_MIMLCP6	(15<<16)
#define	TMGSEL0_HPBIF3_ADG	(0<<24)
#define	TMGSEL0_HPBIF3_SSI0_WS	(1<<24)
#define	TMGSEL0_HPBIF3_SSI1_WS	(2<<24)
#define	TMGSEL0_HPBIF3_SSI2_WS	(3<<24)
#define	TMGSEL0_HPBIF3_SSI3_WS	(4<<24)
#define	TMGSEL0_HPBIF3_SSI4_WS	(5<<24)
#define	TMGSEL0_HPBIF3_SSI5_WS	(6<<24)
#define	TMGSEL0_HPBIF3_SSI6_WS	(7<<24)
#define	TMGSEL0_HPBIF3_SSI7_WS	(8<<24)
#define	TMGSEL0_HPBIF3_MIMLCP0	(9<<24)
#define	TMGSEL0_HPBIF3_MIMLCP1	(10<<24)
#define	TMGSEL0_HPBIF3_MIMLCP2	(11<<24)
#define	TMGSEL0_HPBIF3_MIMLCP3	(12<<24)
#define	TMGSEL0_HPBIF3_MIMLCP4	(13<<24)
#define	TMGSEL0_HPBIF3_MIMLCP5	(14<<24)
#define	TMGSEL0_HPBIF3_MIMLCP6	(15<<24)

/* SRC_TIMING_SELECT1 */
#define	TMGSEL1_HPBIF4_ADG	(0<<0)
#define	TMGSEL1_HPBIF4_SSI0_WS	(1<<0)
#define	TMGSEL1_HPBIF4_SSI1_WS	(2<<0)
#define	TMGSEL1_HPBIF4_SSI2_WS	(3<<0)
#define	TMGSEL1_HPBIF4_SSI3_WS	(4<<0)
#define	TMGSEL1_HPBIF4_SSI4_WS	(5<<0)
#define	TMGSEL1_HPBIF4_SSI5_WS	(6<<0)
#define	TMGSEL1_HPBIF4_SSI6_WS	(7<<0)
#define	TMGSEL1_HPBIF4_SSI7_WS	(8<<0)
#define	TMGSEL1_HPBIF4_MIMLCP0	(9<<0)
#define	TMGSEL1_HPBIF4_MIMLCP1	(10<<0)
#define	TMGSEL1_HPBIF4_MIMLCP2	(11<<0)
#define	TMGSEL1_HPBIF4_MIMLCP3	(12<<0)
#define	TMGSEL1_HPBIF4_MIMLCP4	(13<<0)
#define	TMGSEL1_HPBIF4_MIMLCP5	(14<<0)
#define	TMGSEL1_HPBIF4_MIMLCP6	(15<<0)
#define	TMGSEL1_HPBIF5_ADG	(0<<8)
#define	TMGSEL1_HPBIF5_SSI0_WS	(1<<8)
#define	TMGSEL1_HPBIF5_SSI1_WS	(2<<8)
#define	TMGSEL1_HPBIF5_SSI2_WS	(3<<8)
#define	TMGSEL1_HPBIF5_SSI3_WS	(4<<8)
#define	TMGSEL1_HPBIF5_SSI4_WS	(5<<8)
#define	TMGSEL1_HPBIF5_SSI5_WS	(6<<8)
#define	TMGSEL1_HPBIF5_SSI6_WS	(7<<8)
#define	TMGSEL1_HPBIF5_SSI7_WS	(8<<8)
#define	TMGSEL1_HPBIF5_MIMLCP0	(9<<8)
#define	TMGSEL1_HPBIF5_MIMLCP1	(10<<8)
#define	TMGSEL1_HPBIF5_MIMLCP2	(11<<8)
#define	TMGSEL1_HPBIF5_MIMLCP3	(12<<8)
#define	TMGSEL1_HPBIF5_MIMLCP4	(13<<8)
#define	TMGSEL1_HPBIF5_MIMLCP5	(14<<8)
#define	TMGSEL1_HPBIF5_MIMLCP6	(15<<8)
#define	TMGSEL1_HPBIF6_ADG	(0<<16)
#define	TMGSEL1_HPBIF6_SSI0_WS	(1<<16)
#define	TMGSEL1_HPBIF6_SSI1_WS	(2<<16)
#define	TMGSEL1_HPBIF6_SSI2_WS	(3<<16)
#define	TMGSEL1_HPBIF6_SSI3_WS	(4<<16)
#define	TMGSEL1_HPBIF6_SSI4_WS	(5<<16)
#define	TMGSEL1_HPBIF6_SSI5_WS	(6<<16)
#define	TMGSEL1_HPBIF6_SSI6_WS	(7<<16)
#define	TMGSEL1_HPBIF6_SSI7_WS	(8<<16)
#define	TMGSEL1_HPBIF6_MIMLCP0	(9<<16)
#define	TMGSEL1_HPBIF6_MIMLCP1	(10<<16)
#define	TMGSEL1_HPBIF6_MIMLCP2	(11<<16)
#define	TMGSEL1_HPBIF6_MIMLCP3	(12<<16)
#define	TMGSEL1_HPBIF6_MIMLCP4	(13<<16)
#define	TMGSEL1_HPBIF6_MIMLCP5	(14<<16)
#define	TMGSEL1_HPBIF6_MIMLCP6	(15<<16)
#define	TMGSEL1_HPBIF7_ADG	(0<<24)
#define	TMGSEL1_HPBIF7_SSI0_WS	(1<<24)
#define	TMGSEL1_HPBIF7_SSI1_WS	(2<<24)
#define	TMGSEL1_HPBIF7_SSI2_WS	(3<<24)
#define	TMGSEL1_HPBIF7_SSI3_WS	(4<<24)
#define	TMGSEL1_HPBIF7_SSI4_WS	(5<<24)
#define	TMGSEL1_HPBIF7_SSI5_WS	(6<<24)
#define	TMGSEL1_HPBIF7_SSI6_WS	(7<<24)
#define	TMGSEL1_HPBIF7_SSI7_WS	(8<<24)
#define	TMGSEL1_HPBIF7_MIMLCP0	(9<<24)
#define	TMGSEL1_HPBIF7_MIMLCP1	(10<<24)
#define	TMGSEL1_HPBIF7_MIMLCP2	(11<<24)
#define	TMGSEL1_HPBIF7_MIMLCP3	(12<<24)
#define	TMGSEL1_HPBIF7_MIMLCP4	(13<<24)
#define	TMGSEL1_HPBIF7_MIMLCP5	(14<<24)
#define	TMGSEL1_HPBIF7_MIMLCP6	(15<<24)

/* SRC_TIMING_SELECT2 */
#define	TMGSEL2_HPBIF8_ADG	(0<<0)
#define	TMGSEL2_HPBIF8_SSI0_WS	(1<<0)
#define	TMGSEL2_HPBIF8_SSI1_WS	(2<<0)
#define	TMGSEL2_HPBIF8_SSI2_WS	(3<<0)
#define	TMGSEL2_HPBIF8_SSI3_WS	(4<<0)
#define	TMGSEL2_HPBIF8_SSI4_WS	(5<<0)
#define	TMGSEL2_HPBIF8_SSI5_WS	(6<<0)
#define	TMGSEL2_HPBIF8_SSI6_WS	(7<<0)
#define	TMGSEL2_HPBIF8_SSI7_WS	(8<<0)
#define	TMGSEL2_HPBIF8_MIMLCP0	(9<<0)
#define	TMGSEL2_HPBIF8_MIMLCP1	(10<<0)
#define	TMGSEL2_HPBIF8_MIMLCP2	(11<<0)
#define	TMGSEL2_HPBIF8_MIMLCP3	(12<<0)
#define	TMGSEL2_HPBIF8_MIMLCP4	(13<<0)
#define	TMGSEL2_HPBIF8_MIMLCP5	(14<<0)
#define	TMGSEL2_HPBIF8_MIMLCP6	(15<<0)

/* SRC_TIMING_SELECT3 */
#define	TMGSEL3_SRCOUT0_SSI3_WS	(0<<0)
#define	TMGSEL3_SRCOUT0_SSI4_WS	(1<<0)
#define	TMGSEL3_SRCOUT0_MIMLCP3	(2<<0)
#define	TMGSEL3_SRCOUT0_MIMLCP4	(3<<0)
#define	TMGSEL3_SRCOUT1_SSI3_WS	(0<<4)
#define	TMGSEL3_SRCOUT1_SSI4_WS	(1<<4)
#define	TMGSEL3_SRCOUT1_MIMLCP3	(2<<4)
#define	TMGSEL3_SRCOUT1_MIMLCP4	(3<<4)
#define	TMGSEL3_SRCOUT2_SSI3_WS	(0<<8)
#define	TMGSEL3_SRCOUT2_SSI4_WS	(1<<8)
#define	TMGSEL3_SRCOUT2_MIMLCP3	(2<<8)
#define	TMGSEL3_SRCOUT2_MIMLCP4	(3<<8)
#define	TMGSEL3_SRCOUT3_SSI3_WS	(0<<12)
#define	TMGSEL3_SRCOUT3_SSI4_WS	(1<<12)
#define	TMGSEL3_SRCOUT3_MIMLCP3	(2<<12)
#define	TMGSEL3_SRCOUT3_MIMLCP4	(3<<12)
#define	TMGSEL3_SRCOUT4_SSI3_WS	(0<<16)
#define	TMGSEL3_SRCOUT4_SSI4_WS	(1<<16)
#define	TMGSEL3_SRCOUT4_MIMLCP3	(2<<16)
#define	TMGSEL3_SRCOUT4_MIMLCP4	(3<<16)

/* HPBIF_MODE */
#define	HPBIF_MD_ACCESS_PIO	(0<<0)
#define	HPBIF_MD_ACCESS_DMA	(1<<0)
#define	HPBIF_MD_WORDSWAP	(1<<8)
#define	HPBIF_MD_SFTNUM_BIT0	(0<<16)
#define	HPBIF_MD_SFTNUM_BIT1	(1<<16)
#define	HPBIF_MD_SFTNUM_BIT2	(2<<16)
#define	HPBIF_MD_SFTNUM_BIT3	(3<<16)
#define	HPBIF_MD_SFTNUM_BIT4	(4<<16)
#define	HPBIF_MD_SFTNUM_BIT5	(5<<16)
#define	HPBIF_MD_SFTNUM_BIT6	(6<<16)
#define	HPBIF_MD_SFTNUM_BIT7	(7<<16)
#define	HPBIF_MD_SFTNUM_BIT8	(8<<16)
#define	HPBIF_MD_SFTNUM_BIT9	(9<<16)
#define	HPBIF_MD_SFTNUM_BIT10	(10<<16)
#define	HPBIF_MD_SFTNUM_BIT11	(11<<16)
#define	HPBIF_MD_SFTNUM_BIT12	(12<<16)
#define	HPBIF_MD_SFTNUM_BIT13	(13<<16)
#define	HPBIF_MD_SFTNUM_BIT14	(14<<16)
#define	HPBIF_MD_SFTNUM_BIT15	(15<<16)
#define	HPBIF_MD_SFTDIR_LEFT	(0<<20)
#define	HPBIF_MD_SFTDIR_RIGHT	(1<<20)

/* SRC_ROUTE_MODE0 */
#define	SRCRT_MD_SRC_DISABLE	(0<<0)
#define	SRCRT_MD_SRC_ENABLE	(1<<0)
#define	SRCRT_MD_UF_ALL0	(1<<16)

/* SRC_ROUTE_CONTROL */
#define	SRCRT_CTRL_START0	(1<<0)
#define	SRCRT_CTRL_START1	(1<<1)
#define	SRCRT_CTRL_START2	(1<<2)
#define	SRCRT_CTRL_START3	(1<<3)
#define	SRCRT_CTRL_START4	(1<<4)
#define	SRCRT_CTRL_START5	(1<<5)
#define	SRCRT_CTRL_START6	(1<<6)
#define	SRCRT_CTRL_START7	(1<<7)
#define	SRCRT_CTRL_START8	(1<<8)
#define	SRCRT_CTRL_START30	(1<<16)
#define	SRCRT_CTRL_START41	(1<<17)

/* SSI_MODE0 bit */
#define	SSI_MODE0_IND0	(1<<0)
#define	SSI_MODE0_IND1	(1<<1)
#define	SSI_MODE0_IND2	(1<<2)
#define	SSI_MODE0_IND3	(1<<3)
#define	SSI_MODE0_IND4	(1<<4)
#define	SSI_MODE0_IND5	(1<<5)
#define	SSI_MODE0_IND6	(1<<6)
#define	SSI_MODE0_IND7	(1<<7)
#define	SSI_MODE0_IND8	(1<<8)
#define	SSI_MODE0_SWAP0	(1<<16)
#define	SSI_MODE0_SWAP1	(1<<17)
#define	SSI_MODE0_SWAP2	(1<<18)
#define	SSI_MODE0_SWAP3	(1<<19)
#define	SSI_MODE0_SWAP4	(1<<20)
#define	SSI_MODE0_SWAP5	(1<<21)
#define	SSI_MODE0_SWAP6	(1<<22)
#define	SSI_MODE0_SWAP7	(1<<23)
#define	SSI_MODE0_SWAP8	(1<<24)

/* SSI_MODE1 bit */
#define	SSI_MODE1_SSI1_IND	(0<<0)	/* SSI1 independent        */
#define	SSI_MODE1_SSI1_SLAVE	(1<<0)	/* SSI1 slave, SSI0 slave  */
#define	SSI_MODE1_SSI1_MASTER	(2<<0)	/* SSI1 slave, SSI0 master */
#define	SSI_MODE1_SSI2_IND	(0<<2)	/* SSI2 independent        */
#define	SSI_MODE1_SSI2_SLAVE	(1<<2)	/* SSI2 slave, SSI0 slave  */
#define	SSI_MODE1_SSI2_MASTER	(2<<2)	/* SSI2 slave, SSI0 master */
#define	SSI_MODE1_SSI012_3MOD	(1<<4)
#define	SSI_MODE1_SSI4_IND	(0<<16)	/* SSI4 independent        */
#define	SSI_MODE1_SSI4_SLAVE	(1<<16)	/* SSI4 slave, SSI3 slave  */
#define	SSI_MODE1_SSI4_MASTER	(2<<16)	/* SSI4 slave, SSI3 master */
#define	SSI_MODE1_SSI34_SYNC	(1<<20)

/* SRC_ADINR */
#define	SRCAI_CHNUM_0		(0<<0)
#define	SRCAI_CHNUM_1		(1<<0)
#define	SRCAI_CHNUM_2		(2<<0)
#define	SRCAI_CHNUM_4		(4<<0)
#define	SRCAI_CHNUM_6		(6<<0)
#define	SRCAI_CHNUM_8		(8<<0)
#define	SRCAI_OTBL_24BIT	(0<<16)
#define	SRCAI_OTBL_22BIT	(2<<16)
#define	SRCAI_OTBL_20BIT	(4<<16)
#define	SRCAI_OTBL_18BIT	(6<<16)
#define	SRCAI_OTBL_16BIT	(8<<16)

/* SRC_IFSVR */
#define	SRC_IFS_FSO	0x00400000ULL	/* 2^22 */
#define	SRC_IFS_44KHZ	44100ULL
#define	SRC_IFS_48KHZ	48000ULL

/* SSICRn bit */
#define	SSICR_EN	(1<<0)
#define	SSICR_TRMD_RX	(0<<1)
#define	SSICR_TRMD_TX	(1<<1)
#define	SSICR_CPEN	(1<<2)
#define	SSICR_MUEN	(1<<3)
#define	SSICR_CKDV_1	(0<<4)
#define	SSICR_CKDV_2	(1<<4)
#define	SSICR_CKDV_4	(2<<4)
#define	SSICR_CKDV_8	(3<<4)
#define	SSICR_CKDV_16	(4<<4)
#define	SSICR_CKDV_6	(5<<4)
#define	SSICR_CKDV_12	(6<<4)
#define	SSICR_BREN	(1<<7)
#define	SSICR_DEL	(1<<8)
#define	SSICR_PDTA	(1<<9)
#define	SSICR_SDTA	(1<<10)
#define	SSICR_SPDP	(1<<11)
#define	SSICR_SWSP	(1<<12)
#define	SSICR_SCKP	(1<<13)
#define	SSICR_M_SLAVE	(0<<14)
#define	SSICR_M_MASTER	(3<<14)
#define	SSICR_SWSD	(1<<14)
#define	SSICR_SCKD	(1<<15)
#define	SSICR_SWL_ST8	(0<<16)
#define	SSICR_SWL_ST16	(1<<16)
#define	SSICR_SWL_ST24	(2<<16)
#define	SSICR_SWL_ST32	(3<<16)
#define	SSICR_SWL_ST48	(4<<16)
#define	SSICR_SWL_ST64	(5<<16)
#define	SSICR_SWL_ST128	(6<<16)
#define	SSICR_SWL_ST256	(7<<16)
#define	SSICR_SWL_MN16	(0<<16)
#define	SSICR_SWL_MN32	(1<<16)
#define	SSICR_SWL_MN48	(2<<16)
#define	SSICR_SWL_MN64	(3<<16)
#define	SSICR_SWL_MN96	(4<<16)
#define	SSICR_SWL_MN128	(5<<16)
#define	SSICR_SWL_MN256	(6<<16)
#define	SSICR_SWL_MN512	(7<<16)
#define	SSICR_DWL_ST8	(0<<19)
#define	SSICR_DWL_ST16	(1<<19)
#define	SSICR_DWL_ST18	(2<<19)
#define	SSICR_DWL_ST20	(3<<19)
#define	SSICR_DWL_ST22	(4<<19)
#define	SSICR_DWL_ST24	(5<<19)
#define	SSICR_DWL_ST32	(6<<19)
#define	SSICR_DWL_MN8	(0<<19)
#define	SSICR_DWL_MN16	(1<<19)
#define	SSICR_CHNL_ST1	(0<<22)
#define	SSICR_CHNL_ST2	(1<<22)
#define	SSICR_CHNL_ST3	(2<<22)
#define	SSICR_CHNL_ST4	(3<<22)
#define	SSICR_CHNL_MN	(0<<22)
#define	SSICR_DIEN	(1<<24)
#define	SSICR_IIEN	(1<<25)
#define	SSICR_OIEN	(1<<26)
#define	SSICR_UIEN	(1<<27)
#define	SSICR_DMEN	(1<<28)
#define	SSICR_FORCE	(1<<31)

/*
 * SSICR setting for AK4643
 *   playback, master, 16bit, stereo
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_P4643_ST	(SSICR_FORCE    | \
			 SSICR_CHNL_ST1 | \
			 SSICR_DWL_ST16 | \
			 SSICR_SWL_ST16 | \
			 SSICR_M_MASTER | \
			 SSICR_SWSP     | \
			 SSICR_DEL      | \
			 SSICR_CKDV_8   | \
			 SSICR_TRMD_TX)

/*
 * SSICR setting for AK4643
 *   playback, master, 16bit, mono(system word=32bit)
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_P4643_MN	(SSICR_FORCE    | \
			 SSICR_CHNL_MN  | \
			 SSICR_DWL_MN16 | \
			 SSICR_SWL_MN32 | \
			 SSICR_M_MASTER | \
			 SSICR_DEL      | \
			 SSICR_CKDV_8  | \
			 SSICR_TRMD_TX)

/*
 * SSICR setting for AK4643
 *   capture, slave, 16bit, stereo
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_C4643_ST	(SSICR_FORCE    | \
			 SSICR_CHNL_ST1 | \
			 SSICR_DWL_ST16 | \
			 SSICR_SWL_ST16 | \
			 SSICR_M_SLAVE  | \
			 SSICR_SWSP     | \
			 SSICR_DEL      | \
			 SSICR_TRMD_RX)

/*
 * SSICR setting for AK4643
 *   capture, slave, 16bit, mono(system word=32bit)
 *   SCLK=256fs(MCLK)/8=32fs
 */
#define	SSICR_C4643_MN	(SSICR_FORCE    | \
			 SSICR_CHNL_MN  | \
			 SSICR_DWL_MN16 | \
			 SSICR_SWL_MN32 | \
			 SSICR_M_SLAVE  | \
			 SSICR_DEL      | \
			 SSICR_TRMD_RX)

/* for SSI start */
#define	SSICR_ENABLE	(SSICR_EN	| \
			 SSICR_OIEN	| \
			 SSICR_UIEN	| \
			 SSICR_DMEN)

/* SSISRn bit */
#define	SSISR_IDST	(1<<0)
#define	SSISR_SWNO	(1<<1)
#define	SSISR_CHNO0	(1<<2)
#define	SSISR_CHNO1	(1<<3)
#define	SSISR_DIRQ	(1<<24)
#define	SSISR_IIRQ	(1<<25)
#define	SSISR_OIRQ	(1<<26)
#define	SSISR_UIRQ	(1<<27)
#define	SSISR_DMRQ	(1<<28)

/* SSIWSRn bit */
#define	SSIWSR_MODE_ST	(0<<0)
#define	SSIWSR_MODE_MN	(1<<0)
#define	SSIWSR_MONO	(1<<1)
#define	SSIWSR_CONT	(1<<8)
#define	SSIWSR_WIDTH_1	(1<<16)
#define	SSIWSR_WIDTH_2	(2<<16)
#define	SSIWSR_WIDTH_3	(3<<16)
#define	SSIWSR_WIDTH_4	(4<<16)
#define	SSIWSR_WIDTH_5	(5<<16)
#define	SSIWSR_WIDTH_6	(6<<16)
#define	SSIWSR_WIDTH_7	(7<<16)
#define	SSIWSR_WIDTH_8	(8<<16)
#define	SSIWSR_WIDTH_9	(9<<16)
#define	SSIWSR_WIDTH_10	(10<<16)
#define	SSIWSR_WIDTH_11	(11<<16)
#define	SSIWSR_WIDTH_12	(12<<16)
#define	SSIWSR_WIDTH_13	(13<<16)
#define	SSIWSR_WIDTH_14	(14<<16)
#define	SSIWSR_WIDTH_15	(15<<16)
#define	SSIWSR_WIDTH_16	(16<<16)
#define	SSIWSR_WIDTH_17	(17<<16)
#define	SSIWSR_WIDTH_18	(18<<16)
#define	SSIWSR_WIDTH_19	(19<<16)
#define	SSIWSR_WIDTH_20	(20<<16)
#define	SSIWSR_WIDTH_21	(21<<16)
#define	SSIWSR_WIDTH_22	(22<<16)
#define	SSIWSR_WIDTH_23	(23<<16)
#define	SSIWSR_WIDTH_24	(24<<16)
#define	SSIWSR_WIDTH_25	(25<<16)
#define	SSIWSR_WIDTH_26	(26<<16)
#define	SSIWSR_WIDTH_27	(27<<16)
#define	SSIWSR_WIDTH_28	(28<<16)
#define	SSIWSR_WIDTH_29	(29<<16)
#define	SSIWSR_WIDTH_30	(30<<16)
#define	SSIWSR_WIDTH_31	(31<<16)

/*
 * SSIWS setting
 *   playback(master) only, 16bit, stereo
 */
#define	SSIWS_ST	SSIWSR_CONT

/*
 * SSIWS setting
 *   playback(master) only, 16bit, mono(system word=32bit)
 *   SYNC pulse=16bit
 */
#define	SSIWS_MN	(SSIWSR_WIDTH_16 | \
			 SSIWSR_CONT	 | \
			 SSIWSR_MONO	 | \
			 SSIWSR_MODE_MN)

/* ADG information */
#define	ADG_BRRA		0x0000	/* BRGA baud rate set register   */
#define	ADG_BRRB		0x0004	/* BRGB baud rate set register   */
#define	ADG_SSICKR		0x0008	/* clock select register         */
#define	ADG_AUDIO_CLK_SEL0	0x000c	/* AUDIO CLOCK select 0 register */
#define	ADG_AUDIO_CLK_SEL1	0x0010	/* AUDIO CLOCK select 1 register */
#define	ADG_AUDIO_CLK_SEL3	0x0018	/* AUDIO CLOCK select 3 register */
#define	ADG_AUDIO_CLK_SEL4	0x001c	/* AUDIO CLOCK select 4 register */
#define	ADG_AUDIO_CLK_SEL5	0x0020	/* AUDIO CLOCK select 5 register */

/* ADG BRRA bit */
#define	ADG_BRRA_CKS_ACLKA		(0<<8)
#define	ADG_BRRA_CKS_ACLKA_DIV4		(1<<8)
#define	ADG_BRRA_CKS_ACLKA_DIV16	(2<<8)
#define	ADG_BRRA_CKS_ACLKA_DIV64	(3<<8)

/* ADG BRRB bit */
#define	ADG_BRRB_CKS_ACLKB		(0<<8)
#define	ADG_BRRB_CKS_ACLKB_DIV4		(1<<8)
#define	ADG_BRRB_CKS_ACLKB_DIV16	(2<<8)
#define	ADG_BRRB_CKS_ACLKB_DIV64	(3<<8)

/* ADG SSICKR bit */
#define	ADG_SSICK_CLKOUT_BRGA		(0<<31)
#define	ADG_SSICK_CLKOUT_BRGB		(1<<31)
#define	ADG_SSICK_BRGA_AUDIO_CLKA	(0<<20)
#define	ADG_SSICK_BRGA_AUDIO_CLKB	(1<<20)
#define	ADG_SSICK_BRGA_AUDIO_CLKC	(4<<20)
#define	ADG_SSICK_BRGB_AUDIO_CLKA	(0<<16)
#define	ADG_SSICK_BRGB_AUDIO_CLKB	(1<<16)
#define	ADG_SSICK_BRGB_AUDIO_CLKC	(4<<16)

/* ADG AUDIO_CLK_SEL0 bit */
#define	ADG_SEL0_SSI3_DIV1		(0<<30)
#define	ADG_SEL0_SSI3_DIV2		(1<<30)
#define	ADG_SEL0_SSI3_DIV4		(2<<30)
#define	ADG_SEL0_SSI3_ACLK_DIV		(0<<28)
#define	ADG_SEL0_SSI3_ACLK_BRGA		(1<<28)
#define	ADG_SEL0_SSI3_ACLK_BRGB		(2<<28)
#define	ADG_SEL0_SSI3_DIVCLK_FIX	(0<<24)
#define	ADG_SEL0_SSI3_DIVCLK_CLKA	(1<<24)
#define	ADG_SEL0_SSI3_DIVCLK_CLKB	(2<<24)
#define	ADG_SEL0_SSI3_DIVCLK_CLKC	(3<<24)
#define	ADG_SEL0_SSI3_DIVCLK_MLBCLK	(4<<24)
#define	ADG_SEL0_SSI2_DIV1		(0<<22)
#define	ADG_SEL0_SSI2_DIV2		(1<<22)
#define	ADG_SEL0_SSI2_DIV4		(2<<22)
#define	ADG_SEL0_SSI2_ACLK_DIV		(0<<20)
#define	ADG_SEL0_SSI2_ACLK_BRGA		(1<<20)
#define	ADG_SEL0_SSI2_ACLK_BRGB		(2<<20)
#define	ADG_SEL0_SSI2_DIVCLK_FIX	(0<<16)
#define	ADG_SEL0_SSI2_DIVCLK_CLKA	(1<<16)
#define	ADG_SEL0_SSI2_DIVCLK_CLKB	(2<<16)
#define	ADG_SEL0_SSI2_DIVCLK_CLKC	(3<<16)
#define	ADG_SEL0_SSI2_DIVCLK_MLBCLK	(4<<16)
#define	ADG_SEL0_SSI1_DIV1		(0<<14)
#define	ADG_SEL0_SSI1_DIV2		(1<<14)
#define	ADG_SEL0_SSI1_DIV4		(2<<14)
#define	ADG_SEL0_SSI1_ACLK_DIV		(0<<12)
#define	ADG_SEL0_SSI1_ACLK_BRGA		(1<<12)
#define	ADG_SEL0_SSI1_ACLK_BRGB		(2<<12)
#define	ADG_SEL0_SSI1_DIVCLK_FIX	(0<<8)
#define	ADG_SEL0_SSI1_DIVCLK_CLKA	(1<<8)
#define	ADG_SEL0_SSI1_DIVCLK_CLKB	(2<<8)
#define	ADG_SEL0_SSI1_DIVCLK_CLKC	(3<<8)
#define	ADG_SEL0_SSI1_DIVCLK_MLBCLK	(4<<8)
#define	ADG_SEL0_SSI0_DIV1		(0<<6)
#define	ADG_SEL0_SSI0_DIV2		(1<<6)
#define	ADG_SEL0_SSI0_DIV4		(2<<6)
#define	ADG_SEL0_SSI0_ACLK_DIV		(0<<4)
#define	ADG_SEL0_SSI0_ACLK_BRGA		(1<<4)
#define	ADG_SEL0_SSI0_ACLK_BRGB		(2<<4)
#define	ADG_SEL0_SSI0_DIVCLK_FIX	(0<<0)
#define	ADG_SEL0_SSI0_DIVCLK_CLKA	(1<<0)
#define	ADG_SEL0_SSI0_DIVCLK_CLKB	(2<<0)
#define	ADG_SEL0_SSI0_DIVCLK_CLKC	(3<<0)
#define	ADG_SEL0_SSI0_DIVCLK_MLBCLK	(4<<0)

/* ADG AUDIO_CLK_SEL1 bit */
#define	ADG_SEL1_SSI7_DIV1		(0<<30)
#define	ADG_SEL1_SSI7_DIV2		(1<<30)
#define	ADG_SEL1_SSI7_DIV4		(2<<30)
#define	ADG_SEL1_SSI7_ACLK_DIV		(0<<28)
#define	ADG_SEL1_SSI7_ACLK_BRGA		(1<<28)
#define	ADG_SEL1_SSI7_ACLK_BRGB		(2<<28)
#define	ADG_SEL1_SSI7_DIVCLK_FIX	(0<<24)
#define	ADG_SEL1_SSI7_DIVCLK_CLKA	(1<<24)
#define	ADG_SEL1_SSI7_DIVCLK_CLKB	(2<<24)
#define	ADG_SEL1_SSI7_DIVCLK_CLKC	(3<<24)
#define	ADG_SEL1_SSI7_DIVCLK_MLBCLK	(4<<24)
#define	ADG_SEL1_SSI6_DIV1		(0<<22)
#define	ADG_SEL1_SSI6_DIV2		(1<<22)
#define	ADG_SEL1_SSI6_DIV4		(2<<22)
#define	ADG_SEL1_SSI6_ACLK_DIV		(0<<20)
#define	ADG_SEL1_SSI6_ACLK_BRGA		(1<<20)
#define	ADG_SEL1_SSI6_ACLK_BRGB		(2<<20)
#define	ADG_SEL1_SSI6_DIVCLK_FIX	(0<<16)
#define	ADG_SEL1_SSI6_DIVCLK_CLKA	(1<<16)
#define	ADG_SEL1_SSI6_DIVCLK_CLKB	(2<<16)
#define	ADG_SEL1_SSI6_DIVCLK_CLKC	(3<<16)
#define	ADG_SEL1_SSI6_DIVCLK_MLBCLK	(4<<16)
#define	ADG_SEL1_SSI5_DIV1		(0<<14)
#define	ADG_SEL1_SSI5_DIV2		(1<<14)
#define	ADG_SEL1_SSI5_DIV4		(2<<14)
#define	ADG_SEL1_SSI5_ACLK_DIV		(0<<12)
#define	ADG_SEL1_SSI5_ACLK_BRGA		(1<<12)
#define	ADG_SEL1_SSI5_ACLK_BRGB		(2<<12)
#define	ADG_SEL1_SSI5_DIVCLK_FIX	(0<<8)
#define	ADG_SEL1_SSI5_DIVCLK_CLKA	(1<<8)
#define	ADG_SEL1_SSI5_DIVCLK_CLKB	(2<<8)
#define	ADG_SEL1_SSI5_DIVCLK_CLKC	(3<<8)
#define	ADG_SEL1_SSI5_DIVCLK_MLBCLK	(4<<8)
#define	ADG_SEL1_SSI4_DIV1		(0<<6)
#define	ADG_SEL1_SSI4_DIV2		(1<<6)
#define	ADG_SEL1_SSI4_DIV4		(2<<6)
#define	ADG_SEL1_SSI4_ACLK_DIV		(0<<4)
#define	ADG_SEL1_SSI4_ACLK_BRGA		(1<<4)
#define	ADG_SEL1_SSI4_ACLK_BRGB		(2<<4)
#define	ADG_SEL1_SSI4_DIVCLK_FIX	(0<<0)
#define	ADG_SEL1_SSI4_DIVCLK_CLKA	(1<<0)
#define	ADG_SEL1_SSI4_DIVCLK_CLKB	(2<<0)
#define	ADG_SEL1_SSI4_DIVCLK_CLKC	(3<<0)
#define	ADG_SEL1_SSI4_DIVCLK_MLBCLK	(4<<0)

/* ADG AUDIO_CLK_SEL3 bit */
#define	ADG_SEL3_HPBIF3_DIV128		(0<<28)
#define	ADG_SEL3_HPBIF3_DIV256		(1<<28)
#define	ADG_SEL3_HPBIF3_DIV512		(2<<28)
#define	ADG_SEL3_HPBIF3_DIV1024		(3<<28)
#define	ADG_SEL3_HPBIF3_DIV2048		(4<<28)
#define	ADG_SEL3_HPBIF3_CLK_CLKA	(0<<24)
#define	ADG_SEL3_HPBIF3_CLK_CLKB	(1<<24)
#define	ADG_SEL3_HPBIF3_CLK_CLKC	(2<<24)
#define	ADG_SEL3_HPBIF3_CLK_MLBCLK	(3<<24)
#define	ADG_SEL3_HPBIF3_CLK_BRGA	(4<<24)
#define	ADG_SEL3_HPBIF3_CLK_BRGB	(5<<24)
#define	ADG_SEL3_HPBIF2_DIV128		(0<<20)
#define	ADG_SEL3_HPBIF2_DIV256		(1<<20)
#define	ADG_SEL3_HPBIF2_DIV512		(2<<20)
#define	ADG_SEL3_HPBIF2_DIV1024		(3<<20)
#define	ADG_SEL3_HPBIF2_DIV2048		(4<<20)
#define	ADG_SEL3_HPBIF2_CLK_CLKA	(0<<16)
#define	ADG_SEL3_HPBIF2_CLK_CLKB	(1<<16)
#define	ADG_SEL3_HPBIF2_CLK_CLKC	(2<<16)
#define	ADG_SEL3_HPBIF2_CLK_MLBCLK	(3<<16)
#define	ADG_SEL3_HPBIF2_CLK_BRGA	(4<<16)
#define	ADG_SEL3_HPBIF2_CLK_BRGB	(5<<16)
#define	ADG_SEL3_HPBIF1_DIV128		(0<<12)
#define	ADG_SEL3_HPBIF1_DIV256		(1<<12)
#define	ADG_SEL3_HPBIF1_DIV512		(2<<12)
#define	ADG_SEL3_HPBIF1_DIV1024		(3<<12)
#define	ADG_SEL3_HPBIF1_DIV2048		(4<<12)
#define	ADG_SEL3_HPBIF1_CLK_CLKA	(0<<8)
#define	ADG_SEL3_HPBIF1_CLK_CLKB	(1<<8)
#define	ADG_SEL3_HPBIF1_CLK_CLKC	(2<<8)
#define	ADG_SEL3_HPBIF1_CLK_MLBCLK	(3<<8)
#define	ADG_SEL3_HPBIF1_CLK_BRGA	(4<<8)
#define	ADG_SEL3_HPBIF1_CLK_BRGB	(5<<8)
#define	ADG_SEL3_HPBIF0_DIV128		(0<<4)
#define	ADG_SEL3_HPBIF0_DIV256		(1<<4)
#define	ADG_SEL3_HPBIF0_DIV512		(2<<4)
#define	ADG_SEL3_HPBIF0_DIV1024		(3<<4)
#define	ADG_SEL3_HPBIF0_DIV2048		(4<<4)
#define	ADG_SEL3_HPBIF0_CLK_CLKA	(0<<0)
#define	ADG_SEL3_HPBIF0_CLK_CLKB	(1<<0)
#define	ADG_SEL3_HPBIF0_CLK_CLKC	(2<<0)
#define	ADG_SEL3_HPBIF0_CLK_MLBCLK	(3<<0)
#define	ADG_SEL3_HPBIF0_CLK_BRGA	(4<<0)
#define	ADG_SEL3_HPBIF0_CLK_BRGB	(5<<0)

/* ADG AUDIO_CLK_SEL4 bit */
#define	ADG_SEL4_HPBIF7_DIV128		(0<<28)
#define	ADG_SEL4_HPBIF7_DIV256		(1<<28)
#define	ADG_SEL4_HPBIF7_DIV512		(2<<28)
#define	ADG_SEL4_HPBIF7_DIV1024		(3<<28)
#define	ADG_SEL4_HPBIF7_DIV2048		(4<<28)
#define	ADG_SEL4_HPBIF7_CLK_CLKA	(0<<24)
#define	ADG_SEL4_HPBIF7_CLK_CLKB	(1<<24)
#define	ADG_SEL4_HPBIF7_CLK_CLKC	(2<<24)
#define	ADG_SEL4_HPBIF7_CLK_MLBCLK	(3<<24)
#define	ADG_SEL4_HPBIF7_CLK_BRGA	(4<<24)
#define	ADG_SEL4_HPBIF7_CLK_BRGB	(5<<24)
#define	ADG_SEL4_HPBIF6_DIV128		(0<<20)
#define	ADG_SEL4_HPBIF6_DIV256		(1<<20)
#define	ADG_SEL4_HPBIF6_DIV512		(2<<20)
#define	ADG_SEL4_HPBIF6_DIV1024		(3<<20)
#define	ADG_SEL4_HPBIF6_DIV2048		(4<<20)
#define	ADG_SEL4_HPBIF6_CLK_CLKA	(0<<16)
#define	ADG_SEL4_HPBIF6_CLK_CLKB	(1<<16)
#define	ADG_SEL4_HPBIF6_CLK_CLKC	(2<<16)
#define	ADG_SEL4_HPBIF6_CLK_MLBCLK	(3<<16)
#define	ADG_SEL4_HPBIF6_CLK_BRGA	(4<<16)
#define	ADG_SEL4_HPBIF6_CLK_BRGB	(5<<16)
#define	ADG_SEL4_HPBIF5_DIV128		(0<<12)
#define	ADG_SEL4_HPBIF5_DIV256		(1<<12)
#define	ADG_SEL4_HPBIF5_DIV512		(2<<12)
#define	ADG_SEL4_HPBIF5_DIV1024		(3<<12)
#define	ADG_SEL4_HPBIF5_DIV2048		(4<<12)
#define	ADG_SEL4_HPBIF5_CLK_CLKA	(0<<8)
#define	ADG_SEL4_HPBIF5_CLK_CLKB	(1<<8)
#define	ADG_SEL4_HPBIF5_CLK_CLKC	(2<<8)
#define	ADG_SEL4_HPBIF5_CLK_MLBCLK	(3<<8)
#define	ADG_SEL4_HPBIF5_CLK_BRGA	(4<<8)
#define	ADG_SEL4_HPBIF5_CLK_BRGB	(5<<8)
#define	ADG_SEL4_HPBIF4_DIV128		(0<<4)
#define	ADG_SEL4_HPBIF4_DIV256		(1<<4)
#define	ADG_SEL4_HPBIF4_DIV512		(2<<4)
#define	ADG_SEL4_HPBIF4_DIV1024		(3<<4)
#define	ADG_SEL4_HPBIF4_DIV2048		(4<<4)
#define	ADG_SEL4_HPBIF4_CLK_CLKA	(0<<0)
#define	ADG_SEL4_HPBIF4_CLK_CLKB	(1<<0)
#define	ADG_SEL4_HPBIF4_CLK_CLKC	(2<<0)
#define	ADG_SEL4_HPBIF4_CLK_MLBCLK	(3<<0)
#define	ADG_SEL4_HPBIF4_CLK_BRGA	(4<<0)
#define	ADG_SEL4_HPBIF4_CLK_BRGB	(5<<0)

/* ADG AUDIO_CLK_SEL5 bit */
#define	ADG_SEL5_HPBIF8_DIV128		(0<<4)
#define	ADG_SEL5_HPBIF8_DIV256		(1<<4)
#define	ADG_SEL5_HPBIF8_DIV512		(2<<4)
#define	ADG_SEL5_HPBIF8_DIV1024		(3<<4)
#define	ADG_SEL5_HPBIF8_DIV2048		(4<<4)
#define	ADG_SEL5_HPBIF8_CLK_CLKA	(0<<0)
#define	ADG_SEL5_HPBIF8_CLK_CLKB	(1<<0)
#define	ADG_SEL5_HPBIF8_CLK_CLKC	(2<<0)
#define	ADG_SEL5_HPBIF8_CLK_MLBCLK	(3<<0)
#define	ADG_SEL5_HPBIF8_CLK_BRGA	(4<<0)
#define	ADG_SEL5_HPBIF8_CLK_BRGB	(5<<0)

/************************************************************************

	structure

************************************************************************/
struct sru_regs {
	u32	src_route_select;
	u32	cmd_route_select;
	u32	src_timing_select0;
	u32	src_timing_select1;
	u32	src_timing_select2;
	u32	src_timing_select3;
	u32	rsv1[2];
	u32	hpbif_mode0;
	u32	hpbif_mode1;
	u32	hpbif_mode2;
	u32	hpbif_mode3;
	u32	hpbif_mode4;
	u32	hpbif_mode5;
	u32	hpbif_mode6;
	u32	hpbif_mode7;
	u32	hpbif_mode8;
	u32	rsv2[3];
	u32	src_route0_mode0;
	u32	src_route0_mode1;
	u32	src_route1_mode0;
	u32	src_route1_mode1;
	u32	src_route2_mode0;
	u32	src_route2_mode1;
	u32	src_route3_mode0;
	u32	src_route3_mode1;
	u32	src_route4_mode0;
	u32	src_route4_mode1;
	u32	src_route5_mode0;
	u32	src_route5_mode1;
	u32	src_route6_mode0;
	u32	src_route6_mode1;
	u32	src_route7_mode0;
	u32	src_route7_mode1;
	u32	src_route8_mode0;
	u32	src_route8_mode1;
	u32	cmd_route0_mode;
	u32	cmd_route1_mode;
	u32	rsv3[8];
	u32	src_route_control;
	u32	rsv4[3];
	u32	ssi_mode0;
	u32	ssi_mode1;
	u32	ssi_control;
	u32	rsv5;
	u32	hpbif_status;
	u32	hpbif_int_enable;
	u32	system_status0;
	u32	system_int_enable0;
};

struct src_regs {
	u32	swrsr;
	u32	srcir;
	u32	rsv1[3];	/* reserved */
	u32	adinr;
	u32	rsv2;		/* reserved */
	u32	ifscr;
	u32	ifsvr;
	u32	srccr;
	u32	mnfsr;
	u32	bfssr;
};

struct ctu_regs {
	u32	swrsr;
	u32	ctuir;
	u32	adinr;
	u32	rsv;		/* reserved */
	u32	cpmdr;
	u32	scmdr;
	u32	sv00r;
	u32	sv01r;
	u32	sv02r;
	u32	sv03r;
	u32	sv04r;
	u32	sv05r;
	u32	sv06r;
	u32	sv07r;
	u32	sv10r;
	u32	sv11r;
	u32	sv12r;
	u32	sv13r;
	u32	sv14r;
	u32	sv15r;
	u32	sv16r;
	u32	sv17r;
	u32	sv20r;
	u32	sv21r;
	u32	sv22r;
	u32	sv23r;
	u32	sv24r;
	u32	sv25r;
	u32	sv26r;
	u32	sv27r;
	u32	sv30r;
	u32	sv31r;
	u32	sv32r;
	u32	sv33r;
	u32	sv34r;
	u32	sv35r;
	u32	sv36r;
	u32	sv37r;
};

struct mix_regs {
	u32	swrsr;
	u32	mixir;
	u32	adinr;
	u32	rsv;		/* reserved */
	u32	mixmr;
	u32	mvpdr;
	u32	mdbar;
	u32	mdbbr;
	u32	mdbcr;
	u32	mdbdr;
	u32	mdber;
	u32	mixsr;
};

struct dvc_regs {
	u32	swrsr;
	u32	dvuir;
	u32	adinr;
	u32	rsv;		/* reserved */
	u32	dvucr;
	u32	zcmcr;
	u32	vrctr;
	u32	vrpdr;
	u32	vrdbr;
	u32	vrwtr;
	u32	vol0r;
	u32	vol1r;
	u32	vol2r;
	u32	vol3r;
	u32	vol4r;
	u32	vol5r;
	u32	vol6r;
	u32	vol7r;
	u32	dvuer;
};

struct ssi_regs {
	u32	cr;	/* control register */
	u32	sr;	/* status register */
	u32	tdr;	/* transmit data register */
	u32	rdr;	/* receive data register */
	u32	rsv[4];	/* reserved */
	u32	wsr;	/* WS mode register */
};

struct rcar_pcm_info {
	int id;				/* DAI ID */
	int de_first;			/* for DMA 1st setting */
	int de_start;			/* for DMA start */
	unsigned int period;		/* for ping-pong control */
	unsigned int tran_period;	/* A number of transffered period */
	spinlock_t pcm_lock;		/* for trigger process */
	struct dma_chan *de_chan;
	struct dma_async_tx_descriptor *de_desc;
	dma_cookie_t de_cookie;
	struct hpb_dmae_slave de_param;
	struct tasklet_struct de_tasklet;
	struct rcar_audio_info *ainfo;
	struct rcar_mixer *minfo;
};

struct rcar_audio_info {
	struct sru_regs *srureg;	/* sru common register      */
	struct src_regs *srcreg[2];	/* [0]:playback [1]:capture */
	struct ssi_regs *ssireg[2];	/* [0]:playback [1]:capture */
	struct hpbdma_pregs *dmareg[2];	/* [0]:playback [1]:capture */
};

/************************************************************************
	external prototype declaration
************************************************************************/
extern struct snd_soc_dai_driver sru_soc_dai[];

extern int sru_pcm_hwdep_new(struct snd_card *card, char *id);
extern void rcar_audio_init(int codec_id);

#endif	/* __SRU_PCM_H__ */
