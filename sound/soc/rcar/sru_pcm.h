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
#define	HPBIF_DATA(x)		(0xffda0000 + ((x) * 0x1000))

/* SRU register offset */
#define	SRC_BASE(x)		(0x00000200 + ((x) * 0x40))
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
#define	SSI_BASE(x)		(0x00001000 + ((x) * 0x40))

/* SRC_ROUTE_SELECT bit */
#define	RTSEL_SRC_PLAY(src)	((1<<0) << (4*(src)))	/* HPBIFn -> SRCn -> SSIn */
#define	RTSEL_SRC_CAPT(src)	((2<<0) << (4*(src)))	/* HPBIFn -> SRCn -> SSIn */

/* SRC_TIMING_SELECT0 */
#define	TMGSEL_HPBIF_ADG(hpbif)	0

static inline u32 TMGSEL_HPBIF_SSI_WS(int hpbif, int ssi_chan)
{
	u32 tmg = 1 + ssi_chan;
	int shift = 8 * (hpbif % 4);
	return tmg << shift;
}

static inline u32 TMGSEL_HPBIF_MIMLCP_WS(int hpbif, int mimlcp)
{
	u32 tmg = 9 + mimlcp;
	int shift = 8 * (hpbif % 4);
	return tmg << shift;
}

/* SRC_TIMING_SELECT3 */
#define	TMGSEL3_SRCOUT_SSI3_WS(srcout)	(0 << 4*(srcout))
#define	TMGSEL3_SRCOUT_SSI4_WS(srcout)	(1 << 4*(srcout))
#define	TMGSEL3_SRCOUT_MIMLCP3(srcout)	(2 << 4*(srcout))
#define	TMGSEL3_SRCOUT_MIMLCP4(srcout)	(3 << 4*(srcout))

/* HPBIF_MODE */
#define	HPBIF_MD_ACCESS_PIO	(0<<0)
#define	HPBIF_MD_ACCESS_DMA	(1<<0)
#define	HPBIF_MD_WORDSWAP	(1<<8)
#define	HPBIF_MD_SFTNUM_BIT(x)	((x)<<16)
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
#define	SSI_MODE0_IND(chan)	(1<<(chan))
#define	SSI_MODE0_SWAP(chan)	(1<<((chan)+16))

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
#define	SSIWSR_WIDTH(width)	((width)<<16)

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
#define	ADG_AUDIO_CLK_SEL3	0x0018	/* AUDIO CLOCK select 3 register */


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

#endif	/* __SRU_PCM_H__ */
