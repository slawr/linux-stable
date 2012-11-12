/*
 * drivers/dma/rcar-hpbdma.c
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This file is based on the drivers/dma/shdma.c
 *
 * Renesas SuperH DMA Engine support
 *
 * base is drivers/dma/flsdma.c
 *
 * Copyright (C) 2009 Nobuhiro Iwamatsu <iwamatsu.nobuhiro@renesas.com>
 * Copyright (C) 2009 Renesas Solutions, Inc. All rights reserved.
 * Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * - DMA of SuperH does not have Hardware DMA chain mode.
 * - MAX DMA size is 16MB.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/rcar-hpbdma.h>
#include <mach/irqs.h>

/* DMA descriptor control */
enum hpb_dmae_desc_status {
	DESC_IDLE,
	DESC_PREPARED,
	DESC_SUBMITTED,
	DESC_SETTING,
	DESC_COMPLETED,	/* completed, have to call callback */
	DESC_WAITING,	/* callback called, waiting for ack / re-submit */
};

#define NR_DESCS_PER_CHANNEL 256
#define PLANE_ON 1
#define PLANE_OFF 0

static struct hpb_dmae_device *g_hpbdev;
static const struct hpb_dmae_channel *g_chanp;
static int g_plane_cnt[HPB_DMAC_USE_CHANNELS];
static int g_desc_flag[HPB_DMAC_USE_CHANNELS];

/* A bitmask with bits enough for enum hpb_dmae_slave_chan_id */
static unsigned long hpb_dmae_slave_used[BITS_TO_LONGS(HPB_DMA_SLAVE_NUMBER)];

static void hpb_dmae_chan_ld_cleanup(struct hpb_dmae_chan *hpb_chan, bool all);

static void hpb_dmae_writel(struct hpb_dmae_chan *hpb_dc, u32 data, u32 reg)
{
	__raw_writel(data, hpb_dc->base + reg / sizeof(u32));
}

static u32 hpb_dmae_readl(struct hpb_dmae_chan *hpb_dc, u32 reg)
{
	return __raw_readl(hpb_dc->base + reg / sizeof(u32));
}

static void dmadcmdr_write(struct hpb_dmae_device *hpbdev, u32 data)
{
	__raw_writel(data, hpbdev->chan_reg + DCMDR / sizeof(u32));
}

static void dmahsrstr_write(struct hpb_dmae_device *hpbdev, u32 ch)
{
	__raw_writel(0x01, hpbdev->comm_reg + HSRSTR(ch) / sizeof(u32));
}

static u32 dmadintsr_read(struct hpb_dmae_device *hpbdev, u32 ch)
{
	if (ch < 32)
		return (__raw_readl(hpbdev->comm_reg + DINTSR0 / sizeof(u32))
			 >> ch) & 0x01;
	else
		return (__raw_readl(hpbdev->comm_reg + DINTSR1 / sizeof(u32))
			 >> (ch - 32)) & 0x01;
}

static void dmadintcr_write(struct hpb_dmae_device *hpbdev, u32 ch)
{
	if (ch < 32)
		__raw_writel((0x01 << ch), hpbdev->comm_reg + DINTCR0
			 / sizeof(u32));
	else
		__raw_writel((0x01 << (ch-32)), hpbdev->comm_reg + DINTCR1
			 / sizeof(u32));
}

static void dmaasyncmdr_write(struct hpb_dmae_device *hpbdev, u32 data)
{
	__raw_writel(data, hpbdev->mode_reg);
}

static u32 dmaasyncmdr_read(struct hpb_dmae_device *hpbdev)
{
	return __raw_readl(hpbdev->mode_reg);
}

static void dmae_enable_int(struct hpb_dmae_device *hpbdev, u32 dmach)
{
	int intreg;

	spin_lock_bh(&hpbdev->reg_lock);
	if (dmach < 32) {
		intreg = __raw_readl(hpbdev->comm_reg + DINTMR0 / sizeof(u32));
		__raw_writel(((0x01 << dmach) | intreg),
			hpbdev->comm_reg + DINTMR0 / sizeof(u32));
	} else {
		intreg = __raw_readl(hpbdev->comm_reg + DINTMR1 / sizeof(u32));
		__raw_writel(((0x01 << (dmach-32)) | intreg),
			hpbdev->comm_reg + DINTMR1 / sizeof(u32));
	}
	spin_unlock_bh(&hpbdev->reg_lock);
}

static void dmae_async_reset(struct hpb_dmae_device *hpbdev, u32 data)
{
	u32 rstr;
	int timeout = 10000;	/* 100ms */

	spin_lock_bh(&hpbdev->reg_lock);
	rstr = __raw_readl(hpbdev->reset_reg);
	rstr |= data;
	__raw_writel(rstr, hpbdev->reset_reg);
	rstr = __raw_readl(hpbdev->reset_reg);
	while (((rstr & data) != data) && (timeout--)) {
		udelay(10);
		rstr = __raw_readl(hpbdev->reset_reg);
	}
	if (timeout <= 0)
		dev_err(hpbdev->common.dev, "dmae_async_reset timeout\n");

	rstr &= ~data;
	__raw_writel(rstr, hpbdev->reset_reg);
	spin_unlock_bh(&hpbdev->reg_lock);
}

static void dmae_set_async_mode(struct hpb_dmae_device *hpbdev,
	u32 mask, u32 data)
{
	u32 mode;

	spin_lock_bh(&hpbdev->reg_lock);
	mode = dmaasyncmdr_read(hpbdev);
	mode &= ~mask;
	mode |= data;
	dmaasyncmdr_write(hpbdev, mode);
	spin_unlock_bh(&hpbdev->reg_lock);
}

static void hpb_dmae_ctl_stop(struct hpb_dmae_device *hpbdev)
{
	dmadcmdr_write(hpbdev, DQSPD);
}

static int hpb_dmae_rst(struct hpb_dmae_device *hpbdev)
{
	u32 ch;

	for (ch = HPB_DMA_USE_START_CH; ch < HPB_DMA_USE_END_CH; ch++)
		dmahsrstr_write(hpbdev, ch);

	return 0;
}
#if 0
static bool dmae_is_busy(struct hpb_dmae_chan *hpb_chan)
{
	u32 dstsr = hpb_dmae_readl(hpb_chan, DSTSR);

	if ((dstsr & 0x01) == 0x01)
		return true; /* working */

	return false; /* waiting */
}
#endif
static unsigned int calc_xmit_shift(struct hpb_dmae_chan *hpb_chan)
{
	struct hpb_dmae_device *hpbdev = container_of(hpb_chan->common.device,
						struct hpb_dmae_device, common);
	struct hpb_dmae_pdata *pdata = hpbdev->pdata;

	int cnt;
	int width = hpb_dmae_readl(hpb_chan, DCR);

	if (((width & (0x03<<8)) == (0x00<<8)) && ((width & 0x03) == 0x00))
		cnt = 0; /* 8bit */
	else if (((width & (0x03<<8)) == (0x01<<8)) && ((width & 0x03) == 0x01))
		cnt = 1; /* 16bit */
	else if (((width & (0x03<<8)) == (0x02<<8)) && ((width & 0x03) == 0x02))
		cnt = 2; /* 32bit */
	else
		cnt = 0;

	return pdata->ts_shift[cnt];
}

static void dmae_set_reg0(
	struct hpb_dmae_chan *hpb_chan, struct hpb_dmae_regs *hw)
{
	hpb_dmae_writel(hpb_chan, hw->sar, SAR0);
	hpb_dmae_writel(hpb_chan, hw->dar, DAR0);
	hpb_dmae_writel(hpb_chan, hw->tcr >> hpb_chan->xmit_shift, TCR0);
}

static void dmae_set_reg1(
	struct hpb_dmae_chan *hpb_chan, struct hpb_dmae_regs *hw)
{
	hpb_dmae_writel(hpb_chan, hw->sar, SAR1);
	hpb_dmae_writel(hpb_chan, hw->dar, DAR1);
	hpb_dmae_writel(hpb_chan, hw->tcr >> hpb_chan->xmit_shift, TCR1);
}

static void dmae_start(struct hpb_dmae_chan *hpb_chan)
{
	hpb_dmae_writel(hpb_chan, DMEN, DCMDR);
}

static void dmae_next_start(struct hpb_dmae_chan *hpb_chan)
{
	hpb_dmae_writel(hpb_chan, DNXT | DMEN, DCMDR);
}

static void dmae_halt(struct hpb_dmae_chan *hpb_chan)
{
	hpb_dmae_writel(hpb_chan, DQEND, DCMDR);
	hpb_dmae_writel(hpb_chan, DMSTP, DSTPR);
}

static void dmae_set_port(struct hpb_dmae_chan *hpb_chan, u32 portreg)
{
	hpb_dmae_writel(hpb_chan, portreg, DPTR);
	hpb_chan->xmit_shift = calc_xmit_shift(hpb_chan);
}

static int dmae_set_dcr(struct hpb_dmae_chan *hpb_chan, u32 val)
{
	hpb_dmae_writel(hpb_chan, val, DCR);

	return 0;
}

static dma_cookie_t hpb_dmae_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct hpb_desc *desc = tx_to_hpb_desc(tx), *chunk, *last = desc, *c;
	struct hpb_dmae_chan *hpb_chan = to_hpb_chan(tx->chan);
	dma_async_tx_callback callback = tx->callback;
	dma_cookie_t cookie;

	spin_lock_bh(&hpb_chan->desc_lock);

	cookie = hpb_chan->common.cookie;
	cookie++;
	if (cookie < 0)
		cookie = 1;

	hpb_chan->common.cookie = cookie;
	tx->cookie = cookie;

	/* Mark all chunks of this descriptor as submitted, move to the queue */
	list_for_each_entry_safe(chunk, c, desc->node.prev, node) {
		/*
		 * All chunks are on the global ld_free, so, we have to find
		 * the end of the chain ourselves
		 */
		if (chunk != desc && (chunk->mark == DESC_IDLE ||
				      chunk->async_tx.cookie > 0 ||
				      chunk->async_tx.cookie == -EBUSY ||
				      &chunk->node == &hpb_chan->ld_free))
			break;
		chunk->mark = DESC_SUBMITTED;
		/* Callback goes to the last chunk */
		chunk->async_tx.callback = NULL;
		chunk->cookie = cookie;

		list_move_tail(&chunk->node, &hpb_chan->ld_queue);
		last = chunk;
	}

	last->async_tx.callback = callback;
	last->async_tx.callback_param = tx->callback_param;

	dev_dbg(hpb_chan->dev, "submit #%d@%p on %d: %x[%d] -> %x\n",
		tx->cookie, &last->async_tx, hpb_chan->id,
		desc->hw.sar, desc->hw.tcr, desc->hw.dar);

	spin_unlock_bh(&hpb_chan->desc_lock);

	return cookie;
}

/* Called with desc_lock held */
static struct hpb_desc *hpb_dmae_get_desc(struct hpb_dmae_chan *hpb_chan)
{
	struct hpb_desc *desc;

	list_for_each_entry(desc, &hpb_chan->ld_free, node)
		if (desc->mark != DESC_PREPARED) {
			BUG_ON(desc->mark != DESC_IDLE);
			list_del(&desc->node);
			return desc;
		}

	return NULL;
}

static const struct hpb_dmae_slave_config *hpb_dmae_find_slave(
	struct hpb_dmae_chan *hpb_chan, struct hpb_dmae_slave *param)
{
	struct dma_device *dma_dev = hpb_chan->common.device;
	struct hpb_dmae_device *hpbdev = container_of(dma_dev,
					struct hpb_dmae_device, common);
	struct hpb_dmae_pdata *pdata = hpbdev->pdata;

	int i;
	if (param->slave_id >= HPB_DMA_SLAVE_NUMBER)
		return NULL;

	for (i = 0; i < pdata->slave_num; i++)
		if (pdata->slave[i].id == param->slave_id)
			return pdata->slave + i;

	return NULL;
}
static void dmae_do_tasklet(unsigned long data);

static void hpb_chan_xfer_ld_queue(struct hpb_dmae_chan *hpb_chan)
{
	struct hpb_desc *desc;
	struct hpb_dmae_device *hpbdev = container_of(hpb_chan->common.device,
						struct hpb_dmae_device, common);
	struct hpb_dmae_slave *param = hpb_chan->common.private;
	int id = hpb_chan->id;

	spin_lock_bh(&hpb_chan->desc_lock);

	/* Find the first not transferred desciptor */
	list_for_each_entry(desc, &hpb_chan->ld_queue, node) {
		if (desc->mark == DESC_SUBMITTED) {
			dev_dbg(hpb_chan->dev, "Queue #%d to %d: %u@%x -> %x\n",
				desc->async_tx.cookie, id,
				desc->hw.tcr, desc->hw.sar, desc->hw.dar);

			if (param->config->flags & HPB_DMAE_SET_ASYNC_RESET)
				dmae_async_reset(hpbdev, param->config->rstr);

			/* Get the ld start address from ld_queue */
			if (hpb_chan->tran_mode == TRAN_SINGLE) {
				dmae_set_reg0(hpb_chan, &desc->hw);
				dmae_start(hpb_chan);
			} else { /* TRAN_DOUBLE */
				if (g_plane_cnt[id] == PLANE_ON) {
					dmae_set_reg0(hpb_chan, &desc->hw);
					if (g_desc_flag[id] == PLANE_ON) {
						dev_dbg(hpb_chan->dev,
							"-- first plane --\n");
						dmae_start(hpb_chan);
						g_desc_flag[id] = PLANE_OFF;
					} else {
						dev_dbg(hpb_chan->dev,
							"-- 2 plane --\n");
						dmae_next_start(hpb_chan);
					}
					g_plane_cnt[id] = PLANE_OFF;
				} else { /* g_plane_cnt[id] == PLANE_OFF */
					dmae_set_reg1(hpb_chan, &desc->hw);
					dev_dbg(hpb_chan->dev,
						"-- 1 plane --\n");
					dmae_next_start(hpb_chan);
					g_plane_cnt[id] = PLANE_ON;
				}
				desc->mark = DESC_SETTING;
			}
			break;
		}
	}

	spin_unlock_bh(&hpb_chan->desc_lock);
}

static void dmae_do_tasklet(unsigned long data)
{
	struct hpb_dmae_chan *hpb_chan = (struct hpb_dmae_chan *)data;
	struct hpb_desc *desc;
	int mode = hpb_chan->tran_mode;

	spin_lock(&hpb_chan->desc_lock);
	list_for_each_entry(desc, &hpb_chan->ld_queue, node) {
		if ((desc->mark == DESC_SUBMITTED && mode == TRAN_SINGLE) ||
		    (desc->mark == DESC_SETTING && mode == TRAN_DOUBLE)) {
			dev_dbg(hpb_chan->dev, "done #%d@%p dst 0x%x\n",
				desc->async_tx.cookie, &desc->async_tx,
				desc->hw.dar);
			desc->mark = DESC_COMPLETED;
			break;
		}
	}
	spin_unlock(&hpb_chan->desc_lock);

	/* clean desc */
	hpb_chan_xfer_ld_queue(hpb_chan);
	hpb_dmae_chan_ld_cleanup(hpb_chan, false);
}

static irqreturn_t hpb_dmae_interrupt(int irq, void *data)
{
	u32 ch;
	u32 ch_id;
	irqreturn_t ret = IRQ_NONE;
	struct hpb_dmae_device *hpbdev;
	struct hpb_dmae_chan *hpb_newchan;
	struct hpb_dmae_chan *hpb_chan = (struct hpb_dmae_chan *)data;

	ch_id = irq - g_chanp->ch_irq;  /* DMA Request Channel ID No.*/
	ch = irq - IRQ_DMAC_H(0); /* DMA Channel No.*/

	hpbdev = g_hpbdev;
	hpb_newchan = hpbdev->chan[ch_id];

	/* Check Complete DMA Transfer */
	if (dmadintsr_read(hpbdev, ch) == 0x01) {
		/* Clear Interrupt status */
		dmadintcr_write(hpbdev, ch);
		ret = IRQ_HANDLED;
		tasklet_schedule(&hpb_chan->tasklet);
	}

	return ret;
}

static int hpb_dmae_alloc_chan_resources(struct dma_chan *chan)
{
	struct hpb_dmae_device *hpbdev;
	struct hpb_dmae_slave *param;
	struct hpb_dmae_chan *hpb_chan;
	struct hpb_desc *desc;
	const struct hpb_dmae_channel *chan_pdata;
	struct hpb_dmae_pdata *pdata;
	int ret, i;

	hpbdev = g_hpbdev;
	param = chan->private;
	hpbdev->chan[param->slave_id] = to_hpb_chan(chan);
	hpb_chan = hpbdev->chan[param->slave_id];
	chan_pdata = &hpbdev->pdata->channel[0];
	pdata = hpbdev->pdata;

	pm_runtime_get_sync(hpb_chan->dev);

	/* copy struct dma_device */
	hpb_chan->common.device = &hpbdev->common;
	hpb_chan->dev = hpbdev->common.dev;

	/*
	 * This relies on the guarantee from dmaengine that alloc_chan_resources
	 * never runs concurrently with itself or free_chan_resources.
	 */
	if (param) {
		const struct hpb_dmae_slave_config *cfg;

		cfg = hpb_dmae_find_slave(hpb_chan, param);
		if (!cfg) {
			ret = -EINVAL;
			goto efindslave;
		}
		param->config = cfg;

		if (test_and_set_bit(param->slave_id, hpb_dmae_slave_used)) {
			ret = -EBUSY;
			goto etestused;
		}

		/* detect slave device data */
		for (i = 0; i < pdata->channel_num; i++) {
			if (chan_pdata->s_id == param->slave_id) {
				hpb_chan->id = param->slave_id;
				hpb_chan->base =
					 hpbdev->chan_reg + chan_pdata->offset
					  / sizeof(u32);
				hpb_chan->irq = chan_pdata->ch_irq;
				dev_dbg(hpb_chan->dev,
					 "  Detect Slave device\n");
				dev_dbg(hpb_chan->dev,
					 " -- slave_id          :0x%x\n"
					 , hpb_chan->id);
				dev_dbg(hpb_chan->dev,
					 " -- chan_pdata->offset:0x%x\n"
					 , chan_pdata->offset);
				dev_dbg(hpb_chan->dev,
					 " -- hpb_chan->irq     :%d\n"
					 , hpb_chan->irq);
				break;
			}
			chan_pdata++;
		}

		/* set up channel irq */
		ret = request_irq(hpb_chan->irq,
				&hpb_dmae_interrupt, IRQF_DISABLED,
				hpb_chan->dev_id, hpb_chan);
		if (ret) {
			dev_err(hpbdev->common.dev,
				"DMA channel request_irq error "
				"with return %d\n", ret);
			goto err_no_irq;
		}

		if ((cfg->dcr & (CT | DIP)) == CT ||
		    (cfg->dcr & (CT | DIP)) == DIP) {
			dev_err(hpbdev->common.dev, "DCR setting error");
			ret = -EIO;
			goto err_no_irq;
		} else if ((cfg->dcr & (CT | DIP)) == (CT | DIP))
			hpb_chan->tran_mode = TRAN_DOUBLE;
		else
			hpb_chan->tran_mode = TRAN_SINGLE;

		if (cfg->flags & HPB_DMAE_SET_ASYNC_MODE)
			dmae_set_async_mode(hpbdev, MD_MASK(cfg->dma_ch),
						cfg->mdr);
		dmae_set_dcr(hpb_chan, cfg->dcr);
		dmae_set_port(hpb_chan, cfg->port);
		dmae_enable_int(hpbdev, cfg->dma_ch);

		/* Init DMA tasklet */
		tasklet_init(&hpb_chan->tasklet, dmae_do_tasklet,
				(unsigned long)hpb_chan);
	}

	spin_lock_bh(&hpb_chan->desc_lock);
	while (hpb_chan->descs_allocated < NR_DESCS_PER_CHANNEL) {
		spin_unlock_bh(&hpb_chan->desc_lock);
		desc = kzalloc(sizeof(struct hpb_desc), GFP_KERNEL);
		if (!desc) {
			spin_lock_bh(&hpb_chan->desc_lock);
			break;
		}
		dma_async_tx_descriptor_init(&desc->async_tx,
					&hpb_chan->common);
		desc->async_tx.tx_submit = hpb_dmae_tx_submit;
		desc->mark = DESC_IDLE;

		spin_lock_bh(&hpb_chan->desc_lock);
		list_add(&desc->node, &hpb_chan->ld_free);
		hpb_chan->descs_allocated++;
	}
	spin_unlock_bh(&hpb_chan->desc_lock);

	if (!hpb_chan->descs_allocated) {
		ret = -ENOMEM;
		goto edescalloc;
	}

	return hpb_chan->descs_allocated;

edescalloc:
	if (param)
		clear_bit(param->slave_id, hpb_dmae_slave_used);
etestused:
efindslave:
err_no_irq:
	pm_runtime_put(hpb_chan->dev);
	return ret;
}


/*
 * hpb_dma_free_chan_resources - Free all resources of the channel.
 */
static void hpb_dmae_free_chan_resources(struct dma_chan *chan)
{
	struct hpb_dmae_chan *hpb_chan = to_hpb_chan(chan);
	struct hpb_desc *desc, *_desc;
	LIST_HEAD(list);
	int descs = hpb_chan->descs_allocated;

	dmae_halt(hpb_chan);

	g_plane_cnt[hpb_chan->id] = PLANE_ON;
	g_desc_flag[hpb_chan->id] = PLANE_ON;

	/* Prepared and not submitted descriptors can still be on the queue */
	if (!list_empty(&hpb_chan->ld_queue))
		hpb_dmae_chan_ld_cleanup(hpb_chan, true);

	free_irq(hpb_chan->irq, hpb_chan);

	if (chan->private) {
		/* The caller is holding dma_list_mutex */
		struct hpb_dmae_slave *param = chan->private;
		clear_bit(param->slave_id, hpb_dmae_slave_used);
	}

	spin_lock_bh(&hpb_chan->desc_lock);

	list_splice_init(&hpb_chan->ld_free, &list);
	hpb_chan->descs_allocated = 0;

	spin_unlock_bh(&hpb_chan->desc_lock);

	if (descs > 0)
		pm_runtime_put(hpb_chan->dev);

	list_for_each_entry_safe(desc, _desc, &list, node)
		kfree(desc);
}

/**
 * hpb_dmae_add_desc - get, set up and return one transfer descriptor
 * @hpb_chan:	DMA channel
 * @flags:	DMA transfer flags
 * @dest:	destination DMA address, incremented when direction equals
 *		DMA_DEV_TO_MEM or DMA_MEM_TO_MEM
 * @src:	source DMA address, incremented when direction equals
 *		DMA_MEM_TO_DEV or DMA_MEM_TO_MEM
 * @len:	DMA transfer length
 * @first:	if NULL, set to the current descriptor and cookie set to -EBUSY
 * @direction:	needed for slave DMA to decide which address to keep constant,
 *		equals DMA_MEM_TO_MEM for MEMCPY
 * Returns 0 or an error
 * Locks: called with desc_lock held
 */
static struct hpb_desc *hpb_dmae_add_desc(struct hpb_dmae_chan *hpb_chan,
	unsigned long flags, dma_addr_t *dest, dma_addr_t *src, size_t *len,
	struct hpb_desc **first, enum dma_transfer_direction direction)
{
	struct hpb_desc *new;
	size_t copy_size;

	if (!*len)
		return NULL;

	/* Allocate the link descriptor from the free list */
	new = hpb_dmae_get_desc(hpb_chan);
	if (!new) {
		dev_err(hpb_chan->dev, "No free link descriptor available\n");
		return NULL;
	}

	copy_size = min(*len, (size_t)HPB_DMA_TCR_MAX + 1);
	new->hw.sar = *src;
	new->hw.dar = *dest;
	new->hw.tcr = copy_size;

	if (!*first) {
		/* First desc */
		new->async_tx.cookie = -EBUSY;
		*first = new;
	} else {
		/* Other desc - invisible to the user */
		new->async_tx.cookie = -EINVAL;
	}

	dev_dbg(hpb_chan->dev,
		"chaining (%u/%u)@%x -> %x with %p, cookie %d, hpbift %d\n",
		copy_size, *len, *src, *dest, &new->async_tx,
		new->async_tx.cookie, hpb_chan->xmit_shift);

	new->mark = DESC_PREPARED;
	new->async_tx.flags = flags;
	new->direction = direction;

	*len -= copy_size;
	if (direction == DMA_MEM_TO_MEM || direction == DMA_MEM_TO_DEV)
		*src += copy_size;
	if (direction == DMA_MEM_TO_MEM || direction == DMA_DEV_TO_MEM)
		*dest += copy_size;

	return new;
}

/*
 * hpb_dmae_prep_sg - prepare transfer descriptors from an SG list
 *
 * Common routine for public (MEMCPY) and slave DMA. The MEMCPY case is also
 * converted to scatter-gather to guarantee consistent locking and a correct
 * list manipulation. For slave DMA direction carries the usual meaning, and,
 * logically, the SG list is RAM and the addr variable contains slave address,
 * e.g., the FIFO I/O register. For MEMCPY direction equals DMA_MEM_TO_MEM
 * and the SG list contains only one element and points at the source buffer.
 */
static struct dma_async_tx_descriptor *hpb_dmae_prep_sg(
	struct hpb_dmae_chan *hpb_chan, struct scatterlist *sgl,
	unsigned int sg_len, dma_addr_t *addr,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct scatterlist *sg;
	struct hpb_desc *first = NULL, *new = NULL /* compiler... */;
	LIST_HEAD(tx_list);
	int chunks = 0;
	int i;

	if (!sg_len)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i)
		chunks += (sg_dma_len(sg) + HPB_DMA_TCR_MAX) /
			(HPB_DMA_TCR_MAX + 1);

	/* Have to lock the whole loop to protect against concurrent release */
	spin_lock_bh(&hpb_chan->desc_lock);

	/*
	 * Chaining:
	 * first descriptor is what user is dealing with in all API calls, its
	 *	cookie is at first set to -EBUSY, at tx-submit to a positive
	 *	number
	 * if more than one chunk is needed further chunks have cookie = -EINVAL
	 * the last chunk, if not equal to the first, has cookie = -ENOSPC
	 * all chunks are linked onto the tx_list head with their .node heads
	 *	only during this function, then they are immediately spliced
	 *	back onto the free list in form of a chain
	 */
	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t sg_addr = sg_dma_address(sg);
		size_t len = sg_dma_len(sg);

		if (!len)
			goto err_get_desc;

		do {
			dev_dbg(hpb_chan->dev, "Add SG #%d@%p[%d], dma %llx\n",
				i, sg, len, (unsigned long long)sg_addr);

			if (direction == DMA_DEV_TO_MEM)
				new = hpb_dmae_add_desc(hpb_chan, flags,
						&sg_addr, addr, &len, &first,
						direction);
			else
				new = hpb_dmae_add_desc(hpb_chan, flags,
						addr, &sg_addr, &len, &first,
						direction);
			if (!new)
				goto err_get_desc;

			new->chunks = chunks--;
			list_add_tail(&new->node, &tx_list);
		} while (len);
	}

	if (new != first)
		new->async_tx.cookie = -ENOSPC;

	/* Put them back on the free list, so, they don't get lost */
	list_splice_tail(&tx_list, &hpb_chan->ld_free);

	spin_unlock_bh(&hpb_chan->desc_lock);

	return &first->async_tx;

err_get_desc:
	list_for_each_entry(new, &tx_list, node)
		new->mark = DESC_IDLE;
	list_splice(&tx_list, &hpb_chan->ld_free);

	spin_unlock_bh(&hpb_chan->desc_lock);

	return NULL;
}

static struct dma_async_tx_descriptor *hpb_dmae_prep_memcpy(
	struct dma_chan *chan, dma_addr_t dma_dest, dma_addr_t dma_src,
	size_t len, unsigned long flags)
{
	struct hpb_dmae_chan *hpb_chan;
	struct scatterlist sg;

	if (!chan || !len)
		return NULL;

	chan->private = NULL;

	hpb_chan = to_hpb_chan(chan);

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_src)), len,
		    offset_in_page(dma_src));
	sg_dma_address(&sg) = dma_src;
	sg_dma_len(&sg) = len;

	return hpb_dmae_prep_sg(hpb_chan, &sg, 1, &dma_dest, DMA_MEM_TO_MEM,
			       flags);
}

static struct dma_async_tx_descriptor *hpb_dmae_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct hpb_dmae_slave *param;
	struct hpb_dmae_chan *hpb_chan;
	dma_addr_t slave_addr;

	if (!chan)
		return NULL;

	hpb_chan = to_hpb_chan(chan);
	param = chan->private;
	slave_addr = param->config->addr;

	/* Someone calling slave DMA on a public channel? */
	if (!param || !sg_len) {
		dev_warn(hpb_chan->dev, "%s: bad parameter: %p, %d, %d\n",
			 __func__, param, sg_len, param ? param->slave_id : -1);
		return NULL;
	}

	/*
	 * if (param != NULL), this is a successfully requested slave channel,
	 * therefore param->config != NULL too.
	 */
	return hpb_dmae_prep_sg(hpb_chan, sgl, sg_len, &slave_addr,
			       direction, flags);
}

static int hpb_dmae_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			   unsigned long arg)
{
	struct hpb_dmae_chan *hpb_chan;
	hpb_chan = to_hpb_chan(chan);

	/* Only supports DMA_TERMINATE_ALL */
	if (cmd != DMA_TERMINATE_ALL)
		return -ENXIO;

	if (!chan)
		return -EINVAL;

	dmae_halt(hpb_chan);

	spin_lock_bh(&hpb_chan->desc_lock);
	if (!list_empty(&hpb_chan->ld_queue)) {
		/* Record partial transfer */
		struct hpb_desc *desc = list_entry(hpb_chan->ld_queue.next,
						  struct hpb_desc, node);
		desc->partial = (desc->hw.tcr - hpb_dmae_readl(hpb_chan, TCR0))
			 << hpb_chan->xmit_shift;

	}
	spin_unlock_bh(&hpb_chan->desc_lock);

	hpb_dmae_chan_ld_cleanup(hpb_chan, true);

	return 0;
}

static dma_async_tx_callback __ld_cleanup(struct hpb_dmae_chan *hpb_chan,
				bool all)
{
	struct hpb_desc *desc, *_desc;
	/* Is the "exposed" head of a chain acked? */
	bool head_acked = false;
	dma_cookie_t cookie = 0;
	dma_async_tx_callback callback = NULL;
	void *param = NULL;
	int mode = hpb_chan->tran_mode;

	spin_lock_bh(&hpb_chan->desc_lock);
	list_for_each_entry_safe(desc, _desc, &hpb_chan->ld_queue, node) {
		struct dma_async_tx_descriptor *tx = &desc->async_tx;

		BUG_ON(tx->cookie > 0 && tx->cookie != desc->cookie);
		BUG_ON(desc->mark != DESC_SUBMITTED &&
		       desc->mark != DESC_COMPLETED &&
		       desc->mark != DESC_SETTING &&
		       desc->mark != DESC_WAITING);

		/*
		 * queue is ordered, and we use this loop to (1) clean up all
		 * completed descriptors, and to (2) update descriptor flags of
		 * any chunks in a (partially) completed chain
		 */
		if ((!all && desc->mark == DESC_SUBMITTED &&
		    desc->cookie != cookie && mode == TRAN_SINGLE) ||
		    (!all && desc->mark == DESC_SETTING &&
		    desc->cookie != cookie && mode == TRAN_DOUBLE))
			break;

		if (tx->cookie > 0)
			cookie = tx->cookie;

		if (desc->mark == DESC_COMPLETED && desc->chunks == 1) {
			if (hpb_chan->completed_cookie != desc->cookie - 1)
				dev_dbg(hpb_chan->dev,
					"Completing cookie %d, expected %d\n",
					desc->cookie,
					hpb_chan->completed_cookie + 1);

			hpb_chan->completed_cookie = desc->cookie;
		}

		/* Call callback on the last chunk */
		if (desc->mark == DESC_COMPLETED && tx->callback) {
			desc->mark = DESC_WAITING;
			callback = tx->callback;
			param = tx->callback_param;
			dev_dbg(hpb_chan->dev,
				"descriptor #%d@%p on %d callback\n",
				tx->cookie, tx, hpb_chan->id);
			BUG_ON(desc->chunks != 1);
			break;
		}

		if (tx->cookie > 0 || tx->cookie == -EBUSY) {
			if (desc->mark == DESC_COMPLETED) {
				BUG_ON(tx->cookie < 0);
				desc->mark = DESC_WAITING;
			}
			head_acked = async_tx_test_ack(tx);
		} else {
			switch (desc->mark) {
			case DESC_COMPLETED:
				desc->mark = DESC_WAITING;
				/* Fall through */
			case DESC_WAITING:
				if (head_acked)
					async_tx_ack(&desc->async_tx);
			}
		}

		dev_dbg(hpb_chan->dev, "descriptor %p #%d completed.\n",
			tx, tx->cookie);

		if (((desc->mark == DESC_COMPLETED ||
		      desc->mark == DESC_WAITING) &&
		     async_tx_test_ack(&desc->async_tx)) || all) {
			/* Remove from ld_queue list */
			desc->mark = DESC_IDLE;
			list_move(&desc->node, &hpb_chan->ld_free);
		}
	}
	spin_unlock_bh(&hpb_chan->desc_lock);

	if (callback)
		callback(param);

	return callback;
}

/*
 * hpb_chan_ld_cleanup - Clean up link descriptors
 *
 * This function cleans up the ld_queue of DMA channel.
 */
static void hpb_dmae_chan_ld_cleanup(struct hpb_dmae_chan *hpb_chan, bool all)
{
	while (__ld_cleanup(hpb_chan, all))
		;

	if (all)
		/* Terminating - forgive uncompleted cookies */
		hpb_chan->completed_cookie = hpb_chan->common.cookie;
}

static void hpb_dmae_memcpy_issue_pending(struct dma_chan *chan)
{
	struct hpb_dmae_chan *hpb_chan = to_hpb_chan(chan);
	hpb_chan_xfer_ld_queue(hpb_chan);
}

static enum dma_status hpb_dmae_tx_status(struct dma_chan *chan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	struct hpb_dmae_chan *hpb_chan = to_hpb_chan(chan);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;
	enum dma_status status;

	hpb_dmae_chan_ld_cleanup(hpb_chan, false);

	last_used = chan->cookie;
	last_complete = hpb_chan->completed_cookie;
	BUG_ON(last_complete < 0);
	dma_set_tx_state(txstate, last_complete, last_used, 0);

	spin_lock_bh(&hpb_chan->desc_lock);

	status = dma_async_is_complete(cookie, last_complete, last_used);

	/*
	 * If we don't find cookie on the queue, it has been aborted and we have
	 * to report error
	 */
	if (status != DMA_SUCCESS) {
		struct hpb_desc *desc;
		status = DMA_ERROR;
		list_for_each_entry(desc, &hpb_chan->ld_queue, node)
			if (desc->cookie == cookie) {
				status = DMA_IN_PROGRESS;
				break;
			}
	}

	spin_unlock_bh(&hpb_chan->desc_lock);

	return status;
}

static int __devinit hpb_dmae_chan_probe(struct hpb_dmae_device *hpbdev, int id)
{
	struct hpb_dmae_chan *new_hpb_chan;
	struct platform_device *pdev = to_platform_device(hpbdev->common.dev);

	/* alloc channel */
	new_hpb_chan = kzalloc(sizeof(struct hpb_dmae_chan), GFP_KERNEL);
	if (!new_hpb_chan) {
		dev_err(hpbdev->common.dev,
			"No free memory for allocating dma channels!\n");
		return -ENOMEM;
	}

	/* copy struct dma_device */
	new_hpb_chan->common.device = &hpbdev->common;
	new_hpb_chan->dev = hpbdev->common.dev;
	new_hpb_chan->id = id;

	g_plane_cnt[new_hpb_chan->id] = PLANE_ON;
	g_desc_flag[new_hpb_chan->id] = PLANE_ON;

	if (pdev->id >= 0)
		snprintf(new_hpb_chan->dev_id, sizeof(new_hpb_chan->dev_id),
			 "hpb-dmae%d.%d", pdev->id, new_hpb_chan->id);
	else
		snprintf(new_hpb_chan->dev_id, sizeof(new_hpb_chan->dev_id),
			 "hpb-dma%d", new_hpb_chan->id);

	spin_lock_init(&new_hpb_chan->desc_lock);

	/* Init descripter manage list */
	INIT_LIST_HEAD(&new_hpb_chan->ld_queue);
	INIT_LIST_HEAD(&new_hpb_chan->ld_free);

	/* Add the channel to DMA device channel list */
	list_add_tail(&new_hpb_chan->common.device_node,
			&hpbdev->common.channels);

	hpbdev->common.chancnt++;

	hpbdev->chan[id] = new_hpb_chan;

	return 0;
}

static void hpb_dmae_chan_remove(struct hpb_dmae_device *hpbdev)
{
	int i;

	for (i = hpbdev->common.chancnt - 1 ; i >= 0 ; i--) {
		if (hpbdev->chan[i]) {
			struct hpb_dmae_chan *hpb_chan = hpbdev->chan[i];

			free_irq(hpb_chan->irq, hpb_chan);

			list_del(&hpb_chan->common.device_node);
			kfree(hpb_chan);
			hpbdev->chan[i] = NULL;
		}
	}
	hpbdev->common.chancnt = 0;
}

static int __init hpb_dmae_probe(struct platform_device *pdev)
{
	struct hpb_dmae_pdata *pdata = pdev->dev.platform_data;
	int err, i;
	struct hpb_dmae_device *hpbdev;
	struct resource *chan, *comm, *rest, *mode, *irq_res;

	/* get platform data */
	if (!pdata || !pdata->channel_num)
		return -ENODEV;

	chan = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	comm = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	rest = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	mode = platform_get_resource(pdev, IORESOURCE_MEM, 3);

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!chan || !irq_res)
		return -ENODEV;

	if (!request_mem_region(chan->start, resource_size(chan), pdev->name)) {
		dev_err(&pdev->dev, "DMAC register region already claimed\n");
		return -EBUSY;
	}

	if (!request_mem_region(comm->start, resource_size(comm), pdev->name)) {
		dev_err(&pdev->dev,
			"DMAC Common Register region already claimed\n");
		err = -EBUSY;
		goto ermrcomm;
	}

	if (!request_mem_region(rest->start, resource_size(rest), pdev->name)) {
		dev_err(&pdev->dev,
			"DMAC Async reset Register region already claimed\n");
		err = -EBUSY;
		goto ermrrest;
	}

	if (!request_mem_region(mode->start, resource_size(mode), pdev->name)) {
		dev_err(&pdev->dev,
			"DMAC Async mode Register region already claimed\n");
		err = -EBUSY;
		goto ermrmode;
	}

	err = -ENOMEM;
	hpbdev = kzalloc(sizeof(struct hpb_dmae_device), GFP_KERNEL);
	if (!hpbdev) {
		dev_err(&pdev->dev, "Not enough memory\n");
		goto ealloc;
	}

	spin_lock_init(&hpbdev->reg_lock);

	hpbdev->chan_reg = ioremap(chan->start, resource_size(chan));
	if (!hpbdev->chan_reg)
		goto emapchan;

	hpbdev->comm_reg = ioremap(comm->start, resource_size(comm));
	if (!hpbdev->comm_reg)
		goto emapcomm;

	hpbdev->reset_reg = ioremap(rest->start, resource_size(rest));
	if (!hpbdev->reset_reg)
		goto emaprest;

	hpbdev->mode_reg = ioremap(mode->start, resource_size(mode));
	if (!hpbdev->mode_reg)
		goto emapmode;

	/* platform data */
	hpbdev->pdata = pdata;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* reset dma controller */
	err = hpb_dmae_rst(hpbdev);
	if (err)
		goto rst_err;

	INIT_LIST_HEAD(&hpbdev->common.channels);

	dma_cap_set(DMA_MEMCPY, hpbdev->common.cap_mask);

	if (comm)
		dma_cap_set(DMA_SLAVE, hpbdev->common.cap_mask);

	hpbdev->common.device_alloc_chan_resources
		= hpb_dmae_alloc_chan_resources;
	hpbdev->common.device_free_chan_resources
		 = hpb_dmae_free_chan_resources;
	hpbdev->common.device_prep_dma_memcpy = hpb_dmae_prep_memcpy;
	hpbdev->common.device_tx_status = hpb_dmae_tx_status;
	hpbdev->common.device_issue_pending = hpb_dmae_memcpy_issue_pending;

	/* Compulsory for DMA_SLAVE fields */
	hpbdev->common.device_prep_slave_sg = hpb_dmae_prep_slave_sg;
	hpbdev->common.device_control = hpb_dmae_control;
	hpbdev->common.dev = &pdev->dev;

	/* Create DMA Channel */
	for (i = 0; i < pdata->channel_num; i++)
		hpb_dmae_chan_probe(hpbdev, i);

	pm_runtime_put(&pdev->dev);

	platform_set_drvdata(pdev, hpbdev);
	dma_async_device_register(&hpbdev->common);

	g_hpbdev = hpbdev;
	g_chanp = &hpbdev->pdata->channel[0];

	return err;

rst_err:
	pm_runtime_put(&pdev->dev);
	iounmap(hpbdev->mode_reg);
emapmode:
	iounmap(hpbdev->reset_reg);
emaprest:
	iounmap(hpbdev->comm_reg);
emapcomm:
	iounmap(hpbdev->chan_reg);
emapchan:
	kfree(hpbdev);
ealloc:
	release_mem_region(mode->start, resource_size(mode));
ermrmode:
	release_mem_region(rest->start, resource_size(rest));
ermrrest:
	release_mem_region(comm->start, resource_size(comm));
ermrcomm:
	release_mem_region(chan->start, resource_size(chan));

	return err;
}

static int __exit hpb_dmae_remove(struct platform_device *pdev)
{
	struct hpb_dmae_device *hpbdev = platform_get_drvdata(pdev);
	struct resource *res;
	int irq = platform_get_irq(pdev, 0);

	dma_async_device_unregister(&hpbdev->common);

	if (irq > 0)
		free_irq(irq, hpbdev);

	/* channel data remove */
	hpb_dmae_chan_remove(hpbdev);

	pm_runtime_disable(&pdev->dev);

	iounmap(hpbdev->mode_reg);
	iounmap(hpbdev->reset_reg);
	iounmap(hpbdev->comm_reg);
	iounmap(hpbdev->chan_reg);

	kfree(hpbdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (res)
		release_mem_region(res->start, resource_size(res));

	return 0;
}

static void hpb_dmae_shutdown(struct platform_device *pdev)
{
	struct hpb_dmae_device *hpbdev = platform_get_drvdata(pdev);
	hpb_dmae_ctl_stop(hpbdev);
}

static struct platform_driver hpb_dmae_driver = {
	.remove		= __exit_p(hpb_dmae_remove),
	.shutdown	= hpb_dmae_shutdown,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "hpb-dma-engine",
	},
};

static int __init hpb_dmae_init(void)
{
	return platform_driver_probe(&hpb_dmae_driver, hpb_dmae_probe);
}
module_init(hpb_dmae_init);

static void __exit hpb_dmae_exit(void)
{
	platform_driver_unregister(&hpb_dmae_driver);
}
module_exit(hpb_dmae_exit);

MODULE_DESCRIPTION("Renesas HPB DMA Engine driver");
MODULE_LICENSE("GPL");
