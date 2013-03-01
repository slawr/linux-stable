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

#define ASYNC_RESET_OFFSET(x)	((x == 20) ? 0 : \
				((x == 21) ? 1 : \
				((x == 22) ? 2 : \
				((x == 23) ? 3 : \
				((x == 24) ? 4 : \
				((x == 25) ? 5 : \
				((x == 26) ? 6 : \
				((x == 27) ? 7 : \
				((x == 39) ? 8 : \
				((x == 40) ? 9 : \
				((x == 41) ? 10 : \
				((x == 43) ? 11 : 0))))))))))))
#define ASYNC_RESET(chan)	(1 << ASYNC_RESET_OFFSET(chan))

#define ASYNC_MD_OFFSET(x)	((x == 20) ? 0 : \
				((x == 21) ? 2 : \
				((x == 22) ? 4 : \
				((x == 23) ? 6 : \
				((x == 25) ? 8 : \
				((x == 26) ? 10 : \
				((x == 27) ? 12 : \
				((x == 39) ? 14 : \
				((x == 40) ? 16 : \
				((x == 41) ? 18 : \
				((x == 24) ? 20 : \
				((x == 43) ? 22 : 0))))))))))))
#define MD_MASK(chan)		(3 << ASYNC_MD_OFFSET(chan))
#define ASMD_SINGLE(chan)	(1 << (ASYNC_MD_OFFSET(chan)+1))
#define ASMD_MULTI(chan)	(0 << (ASYNC_MD_OFFSET(chan)+1))
#define ASBTMD_BURST(chan)	(1 << ASYNC_MD_OFFSET(chan))
#define ASBTMD_NBURST(chan)	(0 << ASYNC_MD_OFFSET(chan))

#define ASYNC_MD_SINGLE		(1 << 0)
#define ASYNC_MD_MULTI		(1 << 1)
#define ASYNC_BTMD_BURST	(1 << 2)
#define ASYNC_BTMD_NBURST	(1 << 3)

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
#define to_hpb_dev(chan) container_of(chan->common.device,\
				     struct hpb_dmae_device, common)

#define TRAN_SINGLE	1
#define TRAN_DOUBLE	2

struct device;

struct hpb_dmae_chan {
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
	int plane_cnt;
	int desc_flag;
};

struct hpb_dmae_chans {
	struct hpb_dmae_chan *rx;
	struct hpb_dmae_chan *tx;
};

struct hpb_dmae_device {
	spinlock_t reg_lock;		/* comm_reg operation lock */
	struct dma_device common;
	struct hpb_dmae_chan *chan[2*HPB_DMAC_USE_CHANNELS];	/* separate tx & rx channels */
	struct hpb_dmae_chans chans[HPB_DMA_USE_END_CH];	/* physical channels */
	struct hpb_dmae_pdata *pdata;
	u32 __iomem *chan_reg;
	u32 __iomem *common_reg;
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
