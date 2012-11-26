/*
 * include/linux/rcar-hpbdma.h
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

#ifndef HPB_DMA_H
#define HPB_DMA_H

#include <linux/list.h>
#include <linux/dma-direction.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>

/* DMA register */
#define SAR0	0x00
#define DAR0	0x04
#define TCR0	0x08
#define SAR1	0x0C
#define DAR1	0x10
#define TCR1	0x14
#define DSASR	0x18
#define DDASR	0x1C
#define DTCSR	0x20
#define DPTR	0x24
#define DCR	0x28
#define		DTAMD	(0x01 << 26)
#define		DTAC	(0x01 << 25)
#define		DTAU	(0x01 << 24)
#define		DTAU1	(0x01 << 23)
#define		SWMD	(0x01 << 22)
#define		BTMD	(0x01 << 21)
#define		PKMD	(0x01 << 20)
#define		CT	(0x01 << 18)
#define		ACMD	(0x01 << 17)
#define		DIP	(0x01 << 16)
#define		SMDL	(0x01 << 13)
#define		SPDAM	(0x01 << 12)
#define		SDRMD_MASK	(0x03 << 10)
#define			SDRMD_MOD	(0x00 << 10)
#define			SDRMD_AUTO	(0x01 << 10)
#define			SDRMD_TIMER	(0x02 << 10)
#define		SPDS_MASK	(0x03 << 8)
#define			SPDS_8BIT	(0x00 << 8)
#define			SPDS_16BIT	(0x01 << 8)
#define			SPDS_32BIT	(0x02 << 8)
#define		DMDL	(0x01 << 5)
#define		DPDAM	(0x01 << 4)
#define		DDRMD_MASK	(0x03 << 2)
#define			DDRMD_MOD	(0x00 << 2)
#define			DDRMD_AUTO	(0x01 << 2)
#define			DDRMD_TIMER	(0x02 << 2)
#define		DPDS_MASK	(0x03)
#define			DPDS_8BIT	(0x00)
#define			DPDS_16BIT	(0x01)
#define			DPDS_32BIT	(0x02)
#define DCMDR	0x2C
#define		BDOUT	(1 << 7)
#define		DQSPD	(1 << 6)
#define		DQSPC	(1 << 5)
#define		DMSPD	(1 << 4)
#define		DMSPC	(1 << 3)
#define		DQEND	(1 << 2)
#define		DNXT	(1 << 1)
#define		DMEN	(1 << 0)
#define DSTPR	0x30
#define		DMSTP	(1 << 0)
#define DSTSR	0x34
#define DDBGR	0x38
#define DDBGR2	0x3C

#define		ASRST41		(1 << 10)	/* SDHI3 */
#define		ASRST40		(1 << 9)	/* SDHI3 */
#define		ASRST39		(1 << 8)	/* SDHI3 */
#define		ASRST27		(1 << 7)	/* SDHI2 */
#define		ASRST26		(1 << 6)	/* SDHI2 */
#define		ASRST25		(1 << 5)	/* SDHI2 */
#define		ASRST23		(1 << 3)	/* SDHI0 */
#define		ASRST22		(1 << 2)	/* SDHI0 */
#define		ASRST21		(1 << 1)	/* SDHI0 */

#define		ASMD41_SINGLE	(1 << 19)	/* SDHI3 */
#define		ASMD41_MULTI	(0 << 19)	/* SDHI3 */
#define		ASBTMD41_BURST	(1 << 18)	/* SDHI3 */
#define		ASBTMD41_NBURST	(0 << 18)	/* SDHI3 */
#define		ASMD40_SINGLE	(1 << 17)	/* SDHI3 */
#define		ASMD40_MULTI	(0 << 17)	/* SDHI3 */
#define		ASBTMD40_BURST	(1 << 16)	/* SDHI3 */
#define		ASBTMD40_NBURST	(0 << 16)	/* SDHI3 */
#define		ASMD39_SINGLE	(1 << 15)	/* SDHI3 */
#define		ASMD39_MULTI	(0 << 15)	/* SDHI3 */
#define		ASBTMD39_BURST	(1 << 14)	/* SDHI3 */
#define		ASBTMD39_NBURST	(0 << 14)	/* SDHI3 */
#define		ASMD27_SINGLE	(1 << 13)	/* SDHI2 */
#define		ASMD27_MULTI	(0 << 13)	/* SDHI2 */
#define		ASBTMD27_BURST	(1 << 12)	/* SDHI2 */
#define		ASBTMD27_NBURST	(0 << 12)	/* SDHI2 */
#define		ASMD26_SINGLE	(1 << 11)	/* SDHI2 */
#define		ASMD26_MULTI	(0 << 11)	/* SDHI2 */
#define		ASBTMD26_BURST	(1 << 10)	/* SDHI2 */
#define		ASBTMD26_NBURST	(0 << 10)	/* SDHI2 */
#define		ASMD25_SINGLE	(1 << 9)	/* SDHI2 */
#define		ASMD25_MULTI	(0 << 9)	/* SDHI2 */
#define		ASBTMD25_BURST	(1 << 8)	/* SDHI2 */
#define		ASBTMD25_NBURST	(0 << 8)	/* SDHI2 */
#define		ASMD23_SINGLE	(1 << 7)	/* SDHI1 */
#define		ASMD23_MULTI	(0 << 7)	/* SDHI1 */
#define		ASBTMD23_BURST	(1 << 6)	/* SDHI1 */
#define		ASBTMD23_NBURST	(0 << 6)	/* SDHI1 */
#define		ASMD22_SINGLE	(1 << 5)	/* SDHI1 */
#define		ASMD22_MULTI	(0 << 5)	/* SDHI1 */
#define		ASBTMD22_BURST	(1 << 4)	/* SDHI1 */
#define		ASBTMD22_NBURST	(0 << 4)	/* SDHI1 */
#define		ASMD21_SINGLE	(1 << 3)	/* SDHI1 */
#define		ASMD21_MULTI	(0 << 3)	/* SDHI1 */
#define		ASBTMD21_BURST	(1 << 2)	/* SDHI1 */
#define		ASBTMD21_NBURST	(0 << 2)	/* SDHI1 */

#define MD_MASK(x)	((x == 21) ? (3 << 2) : \
			((x == 22) ? (3 << 4) : \
			((x == 23) ? (3 << 6) : \
			((x == 25) ? (3 << 8) : \
			((x == 26) ? (3 << 10) : \
			((x == 27) ? (3 << 12) : \
			((x == 39) ? (3 << 14) : \
			((x == 40) ? (3 << 16) : \
			((x == 41) ? (3 << 18) : 0)))))))))

/* DMA common register*/
#define DTIMR			0x00
#define DINTSR0			0x0C
#define DINTSR1			0x10
#define DINTCR0			0x14
#define DINTCR1			0x18
#define DINTMR0			0x1C
#define DINTMR1			0x20
#define DACTSR0			0x24
#define DACTSR1			0x28
#define HSRSTR(n)		(0x40 + (n*0x04)) /* n = 0 - 38 */
#define HPB_DMASPR(n)	(0x140 + (n*0x04))  /* n = 0 - 4 */
#define HPB_DMLVLR0		0x160
#define HPB_DMLVLR1		0x164
#define HPB_DMSHPT0		0x168
#define HPB_DMSHPT1		0x16C

#define HPB_DMA_MAXCH 44
#define HPB_DMA_USE_START_CH 20
#define HPB_DMA_USE_END_CH (HPB_DMA_MAXCH - 1)

#define HPB_DMAC_USE_CHANNELS (HPB_DMA_USE_END_CH - HPB_DMA_USE_START_CH + 1)
#define HPB_DMA_SLAVE_NUMBER 256
#define HPB_DMA_TCR_MAX 0x03FFFFFF	/* 64MB */

#define to_hpb_chan(chan) container_of(chan, struct hpb_dmae_chan, common)
#define to_hpb_desc(lh) container_of(lh, struct hpb_desc, node)
#define tx_to_hpb_desc(tx) container_of(tx, struct hpb_desc, async_tx)

#define TRAN_SINGLE	1
#define TRAN_DOUBLE	2

struct device;

struct hpb_dmae_chan {
	dma_cookie_t completed_cookie;	/* The maximum cookie completed */
	spinlock_t desc_lock;		/* Descriptor operation lock */
	struct list_head ld_queue;	/* Link descriptors queue */
	struct list_head ld_free;	/* Link descriptors free */
	struct dma_chan common;		/* DMA common channel */
	struct device *dev;		/* Channel device */
	struct tasklet_struct tasklet;	/* Tasklet */
	int descs_allocated;		/* desc count */
	int xmit_shift;			/* log_2(bytes_per_xfer) */
	int irq;
	int id;				/* Raw id of this channel */
	int tran_mode;			/* DMA transfer mode */
	u32 __iomem *base;
	char dev_id[16];		/* unique name per DMAC of channel */
};

struct hpb_dmae_device {
	spinlock_t reg_lock;		/* comm_reg operation lock */
	struct dma_device common;
	struct hpb_dmae_chan *chan[HPB_DMAC_USE_CHANNELS];
	struct hpb_dmae_pdata *pdata;
	u32 __iomem *chan_reg;
	u32 __iomem *comm_reg;
	u32 __iomem *reset_reg;
	u32 __iomem *mode_reg;
};


/* Used by slave DMA clients to request DMA to/from a specific peripheral */
struct hpb_dmae_slave {
	unsigned int			slave_id; /* Set by the platform */
	struct device			*dma_dev; /* Set by the platform */
	const struct hpb_dmae_slave_config	*config; /* Set by the driver */
};

struct hpb_dmae_regs {
	u32 sar; /* SAR / source address */
	u32 dar; /* DAR / destination address */
	u32 tcr; /* TCR / transfer count */
};

struct hpb_desc {
	struct hpb_dmae_regs hw;
	struct list_head node;
	struct dma_async_tx_descriptor async_tx;
	enum dma_transfer_direction direction;
	dma_cookie_t cookie;
	size_t partial;
	int chunks;
	int mark;
};

struct hpb_dmae_slave_config {
	unsigned int	id;
	dma_addr_t	addr;
	u32		dcr;
	u32		port;
	u32		rstr;
	u32		mdr;
	u32		flags;
#define	HPB_DMAE_SET_ASYNC_RESET	(1<<0)
#define	HPB_DMAE_SET_ASYNC_MODE		(1<<1)
#define	HPB_DMAE_SET_SHPT1		(1<<2)
	u32		dma_ch;
};

struct hpb_dmae_channel {
	unsigned int	offset;
	unsigned int	ch_irq;
	unsigned int	s_id;
};

struct hpb_dmae_pdata {
	const struct hpb_dmae_slave_config *slave;
	int slave_num;
	const struct hpb_dmae_channel *channel;
	int channel_num;
	const unsigned int *ts_shift;
	int ts_shift_num;
};

#endif
